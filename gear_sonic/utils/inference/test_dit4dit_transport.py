"""End-to-end gateway test on isolated localhost ports. No DDS/robot connection."""
import json, os, pty, subprocess, sys, tempfile, time
from pathlib import Path
import msgpack
import numpy as np
import zmq
from gear_sonic.camera.sensor_server import ImageMessageSchema
from gear_sonic.utils.teleop.zmq.zmq_planner_sender import build_command_message, build_planner_message


def run_scenario(fault):
    ctx=zmq.Context();sockets=[];ports={}
    for name in ('camera','state','dex','pico','policy'):
        s=ctx.socket(zmq.REP if name=='policy' else zmq.PUB);s.setsockopt(zmq.LINGER,0)
        ports[name]=s.bind_to_random_port('tcp://127.0.0.1',min_port=16000,max_port=25000)
        sockets.append(s)
    camera,state,dex,pico,policy=sockets
    for name in ('action','target'):
        tmp=ctx.socket(zmq.PUB)
        ports[name]=tmp.bind_to_random_port('tcp://127.0.0.1',min_port=25001,max_port=32000);tmp.close()
    rx=ctx.socket(zmq.SUB);rx.setsockopt(zmq.SUBSCRIBE,b'');rx.setsockopt(zmq.LINGER,0)
    rx.connect(f'tcp://127.0.0.1:{ports["action"]}');sockets.append(rx)
    assert all(x>=16000 for x in ports.values())
    root=Path(tempfile.mkdtemp(prefix='dit4dit_transport_'));master,slave=pty.openpty()
    out=(root/'stdout.log').open('w')
    cmd=[sys.executable,'-m','gear_sonic.scripts.run_dit4dit_gateway','--execute',
         '--task-prompt','TEST ONLY','--limits','gear_sonic/configs/dex1_limits.g1_20261013.json',
         '--log-dir',str(root/'run'),'--duration','18']
    for flag,name in [('camera','camera'),('state','state'),('dex-telemetry','dex'),('pico','pico'),('policy','policy'),('action','action'),('target','target')]:
        cmd += ['--'+flag+'-port',str(ports[name])]
    child=subprocess.Popen(cmd,stdin=slave,stdout=out,stderr=out);os.close(slave)
    start=time.monotonic();phase=0;phase_at=start;pending=None;seq=0;tokens=0;post_stop_tokens=0;last_command=-1.;fault_active=False
    jpeg=ImageMessageSchema({},dict(ego_view=np.zeros((32,32,3),dtype=np.uint8))).serialize()['images']
    try:
        while time.monotonic()-start<17:
            now=time.monotonic();seq+=1
            if child.poll() is not None:
                raise AssertionError((root/'stdout.log').read_text()[-3000:])
            if not (fault_active and fault=='camera'):
                camera.send(msgpack.packb(dict(timestamps=dict(ego_view=time.time()),images=jpeg),use_bin_type=True))
            state.send(b'g1_debug'+msgpack.packb(dict(body_q=[0.]*29),use_bin_type=True))
            stop=fault_active and fault=='bottom'
            state.send(b'dex1_control_status '+msgpack.packb(dict(seq=seq,time=now,state='STOP' if stop else 'CONTROL',stop=stop),use_bin_type=True))
            sides={s:dict(q=1.,dq=0.,feedback_time=now,feedback_valid=True) for s in ('left','right')}
            dex.send(b'dex1_telemetry '+msgpack.packb(dict(time=now,phase=4,sides=sides),use_bin_type=True))
            if phase==0 and now-last_command>.1:
                pico.send(build_command_message(False,False,True));last_command=now
            pico.send(build_planner_message(0,[0,0,0],[1,0,0]))
            pico.send(b'dex1_trigger '+msgpack.packb(dict(seq=seq,time=now,sample_time=now,left=0.,right=0.,valid=True,stop=False),use_bin_type=True))
            if pending is None and policy.poll(0):
                req=policy.recv_json();pending=(now,req)
            if pending and now-pending[0]>.05 and not (fault_active and fault=='timeout'):
                req=pending[1]
                policy.send_json(dict(ok=True,generation=req['generation'],request_id=req['request_id'],observed_at=req['observed_at'],actions=np.zeros((50,66)).tolist()))
                pending=None
            while rx.poll(0):
                raw=rx.recv()
                if raw.startswith(b'pose'):
                    tokens+=1
                    if phase>=3 and now-phase_at>.2:post_stop_tokens+=1
            events=root/'run/events.jsonl'
            rows=[]
            if events.exists():
                for line in events.read_text().splitlines():
                    try:rows.append(json.loads(line))
                    except json.JSONDecodeError:pass
            if phase==0 and any(r['kind']=='prediction' for r in rows):
                assert tokens==0,'readonly emitted tokens'
                os.write(master,b'model\n');phase=1;phase_at=now
            elif phase==1 and tokens>=5:
                if fault=='takeover':os.write(master,b'teleop\n')
                else:fault_active=True
                phase=2;phase_at=now
            elif phase==2:
                expected='teleop' if fault=='takeover' else 'stop'
                if any(r['kind']=='mode' and r.get('mode')==expected for r in rows):
                    phase=3;phase_at=now
                    if fault!='takeover':os.write(master,b'model\n')  # cannot unlatch
            elif phase==3 and now-phase_at>1.2:
                assert post_stop_tokens==0, 'tokens continued after stop/takeover'
                print('PASS',fault,'tokens',tokens,'log',root,flush=True)
                return
            time.sleep(.015)
        raise AssertionError('scenario did not finish: '+str(root)+'\n'+(root/'stdout.log').read_text()[-2500:])
    finally:
        os.write(master,b'quit\n')
        try:child.wait(timeout=4)
        except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=3)
        os.close(master);out.close()
        for s in sockets:s.close()
        ctx.term()

if __name__=='__main__':
    for fault in ('bottom','camera','timeout','takeover'):run_scenario(fault)
