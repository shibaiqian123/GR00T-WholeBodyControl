"""Isolated simulation replay of recorded starts. No policy service or physical-device launcher."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import pty
import signal
import socket
import subprocess
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
WRAPPER=Path('/home/cqu023/sonic-diagnostics/sonic-realtime-v1/run_sim_realtime.py')
RUNS=['model_only_20261028_203518','model_only_20261028_203841','model_only_20261028_203831']

def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(8*1024*1024),b''):h.update(b)
    return h.hexdigest()

def load_cases(root=ROOT):
    cases=[]
    for name in RUNS:
        path=root/'outputs'/name/'gateway/events.jsonl'
        events=[json.loads(s) for s in path.read_text().splitlines()]
        starts=[e for e in events if e['kind']=='bottom_start']
        if len(starts)!=1:raise ValueError(f'Expected one original start: {path}')
        start=starts[0]['time']
        commands=[e for e in events if e['kind']=='command' and e.get('command')=='model' and e['time']<=start]
        switch=commands[-1]['time']
        # Match the original accepted model command, preserving all original frame counters.
        frames=[e for e in events if e['kind']=='frame' and switch<=e['time']<=start+2.3]
        if not frames or not any(e['time']<start for e in frames):raise ValueError('Missing prefill')
        for f in frames:
            a=np.asarray(f['token'])
            if a.shape!=(64,) or not np.isfinite(a).all():raise ValueError('Invalid source token')
        schedule=[dict(kind='switch',time=switch),dict(kind='start',time=start)]+frames
        schedule.sort(key=lambda e:e['time'])
        cases.append(dict(name=name,source=str(path),sha256=sha(path),start=start,switch=switch,schedule=schedule))
    return sorted(cases,key=lambda c:c['start'])

def check_ports():
    for port in (15556,15557):
        with socket.socket() as s:
            s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            s.bind(('127.0.0.1',port))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--manual-ready',action='store_true',help='Wait for an operator-created ready file in each case directory before replay')
    p.add_argument('--execute',action='store_true',help='Run simulation only; omitted is preflight')
    p.add_argument("--max-cases",type=int,choices=(1,2,3),default=3,help="Limit independent cases in monotonic order")
    a=p.parse_args()
    os.chdir(ROOT)
    from gear_sonic.utils.mujoco_sim.configs import SimLoopConfig
    config=SimLoopConfig(interface='lo',dds_domain=38).load_wbc_yaml()
    if config['DOMAIN_ID']!=38 or config['INTERFACE']!='lo':raise ValueError('Isolation mismatch')
    check_ports();cases=load_cases()[:a.max_cases]
    deploy=ROOT/'gear_sonic_deploy'
    binary=deploy/'target/release/g1_deploy_onnx_ref'
    resources=[WRAPPER,binary,deploy/'target/release/replay_sonic_decoder',deploy/'policy/sonic_v1_1/model_decoder.onnx',deploy/'policy/sonic_v1_1/model_encoder.onnx',deploy/'policy/sonic_v1_1/observation_config.yaml',deploy/'planner/target_vel/V2/planner_sonic.onnx']
    for resource in resources:
        if not resource.is_file():raise FileNotFoundError(resource)
    manifest=dict(domain=38,interface='lo',ports=[15556,15557],config=config,
        git_head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        assets={str(x):sha(x) for x in resources},cases=[{k:v for k,v in c.items() if k!='schedule'} for c in cases],
        note='Mechanism replay with default simulation pose/support; not exact physical reconstruction')
    print(json.dumps(manifest,default=str),flush=True)
    if not a.execute:return
    a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'manifest.json').write_text(json.dumps(manifest,default=str,indent=2))
    (a.output/'source.diff').write_text(subprocess.check_output(['git','diff'],text=True))
    results=[]
    for i,case in enumerate(cases):
        results.append(run_case(case,a.output/f'{i+1}_{case["name"]}',binary,deploy,a.manual_ready))
        (a.output/'results.json').write_text(json.dumps(results,indent=2))
        # Never automatically retry a failed case; independent cases still use fresh processes.
    print(json.dumps(results,indent=2),flush=True)

def run_case(case,out,binary,deploy,manual_ready=False):
    import zmq,msgpack
    from gear_sonic.utils.teleop.zmq.zmq_planner_sender import build_command_message,pack_pose_message
    check_ports();out.mkdir()
    env=os.environ.copy()
    env.update(__NV_PRIME_RENDER_OFFLOAD='1',__GLX_VENDOR_LIBRARY_NAME='nvidia',
               DISPLAY=os.environ.get('DISPLAY',':1'),XAUTHORITY=os.environ.get('XAUTHORITY','/run/user/1000/gdm/Xauthority'))
    # Reject inherited transport/control overrides before creating either process.
    for key in ('CYCLONEDDS_URI','VIBE_CONFIG','VIBE_HYBRID_ENDPOINT'):
        if env.get(key):raise ValueError('Unexpected override '+key)
    simcmd=[str(ROOT/'.venv_teleop/bin/python'),'-u',str(WRAPPER),'--interface','lo','--dds-domain','38']
    cmd=[str(binary),'lo','policy/sonic_v1_1/model_decoder.onnx','reference/example/',
        '--obs-config','policy/sonic_v1_1/observation_config.yaml','--encoder-file','policy/sonic_v1_1/model_encoder.onnx',
        '--planner-file','planner/target_vel/V2/planner_sonic.onnx','--input-type','zmq_manager','--output-type','zmq',
        '--zmq-host','127.0.0.1','--zmq-port','15556','--zmq-out-port','15557','--zmq-out-host','127.0.0.1',
        '--dds-domain','38','--disable-crc-check','--enable-dex1-heartbeat','--motor-kp-scale','4,10=1.5','--motor-kd-scale','4,10=1.5']
    (out/'commands.json').write_text(json.dumps(dict(sim=simcmd,bottom=cmd),indent=2))
    ctx=zmq.Context();pub=ctx.socket(zmq.PUB);pub.setsockopt(zmq.LINGER,0);pub.bind('tcp://127.0.0.1:15556')
    sub=ctx.socket(zmq.SUB);sub.setsockopt(zmq.LINGER,0);sub.setsockopt(zmq.SUBSCRIBE,b'dex1_control_status ');sub.connect('tcp://127.0.0.1:15557')
    logs=[(out/'sim.log').open('w'),(out/'bottom.log').open('w')]
    ev=(out/'replay.jsonl').open('w',buffering=1)
    sim=bottom=None;master=slave=None;status=None;control_time=None;failure=None;sent=[]
    def record(kind,**fields):ev.write(json.dumps(dict(time=time.monotonic(),kind=kind,**fields),allow_nan=False)+'\n')
    def poll():
        nonlocal status,control_time
        while True:
            try:raw=sub.recv(zmq.NOBLOCK)
            except zmq.Again:break
            status=msgpack.unpackb(raw[len(b'dex1_control_status '):],raw=False)
            record('heartbeat',packet=status)
            if status.get('state')=='CONTROL' and control_time is None:control_time=time.monotonic()
        for name,proc in [('sim',sim),('bottom',bottom)]:
            if proc and proc.poll() is not None:raise RuntimeError(name+' exited early: '+str(proc.returncode))
    try:
        simenv=dict(env,SONIC_SIM_AUDIT=str(out/'sim.jsonl'))
        sim=subprocess.Popen(simcmd,cwd=ROOT,env=simenv,stdout=logs[0],stderr=subprocess.STDOUT,start_new_session=True)
        master,slave=pty.openpty()
        bottom=subprocess.Popen(cmd,cwd=deploy,env=dict(env,SONIC_STARTUP_AUDIT=str(out/'bottom.jsonl')),stdin=slave,stdout=logs[1],stderr=subprocess.STDOUT,start_new_session=True)
        wait_begin=time.monotonic()
        while True:
            poll()
            if status and status.get('state')=='WAIT_FOR_CONTROL' and time.monotonic()-status['time']<.25:break
            if time.monotonic()-wait_begin>90:raise RuntimeError('WAIT_FOR_CONTROL unavailable')
            time.sleep(.01)
        record('isolation',domain=38,interface='lo',ports=[15556,15557])
        (out/'sockets.txt').write_text(subprocess.check_output(['ss','-tunap'],text=True))
        if manual_ready:
            gate=out/'operator.ready'
            print('WAITING FOR OPERATOR: adjust rope, then create '+str(gate),flush=True)
            record('waiting_operator',ready_file=str(gate))
            while not gate.exists():poll();time.sleep(.02)
            record('operator_ready')
        # Capture a full two-second stable pre-start interval; no commands sent during this wait.
        ready_at=time.monotonic()
        while time.monotonic()-ready_at<2.1:poll();time.sleep(.005)
        # Require recorded runtime support, not only an enabled configuration flag.
        state_rows=[json.loads(line) for line in (out/'sim.jsonl').read_text().splitlines() if line.endswith('}')]
        latest=next((r for r in reversed(state_rows) if r.get('kind')=='sim_state'),None)
        if latest is None or not latest.get('support',{}).get('enabled'):
            raise RuntimeError('Runtime elastic support not verified')
        if not np.isfinite(latest['qpos']).all() or latest['qpos'][2]<.5:
            raise RuntimeError('Simulator unstable before start; no replay')
        record('support_verified',support=latest['support'],sample_time=latest['time'])
        origin=time.monotonic()+.1
        last_sent=None
        for e in case['schedule']:
            due=origin+e['time']-case['switch']
            # Do not burst to catch up: preserve original minimum spacing if a prior send was late.
            if last_sent is not None and e['kind']=='frame':due=max(due,last_sent[0]+e['time']-last_sent[1])
            while time.monotonic()<due:poll();time.sleep(min(.001,max(0,due-time.monotonic())))
            poll();now=time.monotonic()
            if now-(origin+e['time']-case['switch'])>.02:raise RuntimeError('Replay scheduling drift > one 50Hz period; no catch-up')
            if e['kind']=='frame':
                pub.send(pack_pose_message(dict(token_state=np.asarray(e['token'],dtype=np.float64).reshape(1,64),frame_index=np.asarray([e['index']],dtype=np.int64)),version=4))
            else:pub.send(build_command_message(e['kind']=='start',False,False))
            actual=time.monotonic()
            if e['kind']=='frame':last_sent=(now,e['time'])
            row=dict(source=e,scheduled=origin+e['time']-case['switch'],sent=actual,lateness=actual-(origin+e['time']-case['switch']))
            sent.append(row);record('send',**row)
        poll()
        if control_time is None:raise RuntimeError('No CONTROL acknowledgment')
        if time.monotonic()-control_time<2:raise RuntimeError('Original recording insufficient for CONTROL +2s; no invented/repeated tokens')
    except Exception as exc:
        failure=str(exc);record('failure',error=failure)
    finally:
        if bottom and bottom.poll() is None:
            pub.send(build_command_message(False,True,False));record('stop_sent')
            try:bottom.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.write(master,b'o');record('keyboard_stop_sent')
                try:bottom.wait(timeout=5)
                except subprocess.TimeoutExpired:bottom.terminate();failure=failure or 'Bottom did not exit normally';bottom.wait(timeout=5)
        if sim and sim.poll() is None:
            sim.send_signal(signal.SIGINT)
            try:sim.wait(timeout=8)
            except subprocess.TimeoutExpired:sim.terminate();failure=failure or 'Simulator did not exit normally';sim.wait(timeout=5)
        for f in logs:f.close()
        ev.close();pub.close();sub.close();ctx.term()
        for fd in (master,slave):
            if fd is not None:os.close(fd)
    result=dict(case=case['name'],output=str(out),control_time=control_time,failure=failure,sent_frames=sum(x['source']['kind']=='frame' for x in sent))
    for filename,footer in [('bottom.jsonl','audit_end'),('sim.jsonl','sim_audit_end')]:
        try:
            rows=[json.loads(s) for s in (out/filename).open()]
            if rows[-1].get('kind')!=footer or rows[-1].get('dropped')!=0 or rows[-1].get('error'):raise ValueError('Missing footer or dropped/error records')
            if filename=='bottom.jsonl':
                end=rows[-1]
                if end.get('write_errors')!=0 or end.get('accepted')!=len(rows)-1 or end.get('written')!=len(rows)-1:raise ValueError('Missing counters, write errors or incomplete trace')
        except Exception as exc:result['failure']=(result['failure'] or '')+'; '+filename+': '+str(exc)
    if 'Cannot sustain real time' in (out/'sim.log').read_text():result['failure']=(result['failure'] or '')+'; real-time pacing failed'
    result['passed']=not result['failure']
    return result

if __name__=='__main__':main()
