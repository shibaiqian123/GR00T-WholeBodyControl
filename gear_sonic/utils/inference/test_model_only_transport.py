"""Pure-model gateway end-to-end with isolated simulated sockets; never DDS."""
import json,os,pty,subprocess,sys,tempfile,time
from pathlib import Path
import msgpack,numpy as np,zmq
from gear_sonic.camera.sensor_server import ImageMessageSchema
from gear_sonic.utils.inference.readonly_body import LowStateCache,TOPIC
from gear_sonic.utils.inference.dit4dit_control import command_fields

def scenario(fault):
    ctx=zmq.Context();sockets=[];ports={}
    for name in ('camera','body','dex','state','policy'):
        s=ctx.socket(zmq.REP if name=='policy' else zmq.PUB);s.setsockopt(zmq.LINGER,0)
        ports[name]=s.bind_to_random_port('tcp://127.0.0.1',min_port=16000,max_port=25000);sockets.append(s)
    camera,body,dex,state,policy=sockets
    for name in ('action','target'):
        s=ctx.socket(zmq.PUB);ports[name]=s.bind_to_random_port('tcp://127.0.0.1',min_port=25001,max_port=32000);s.close()
    receivers={}
    for name in ('action','target'):
        s=ctx.socket(zmq.SUB);s.setsockopt(zmq.LINGER,0);s.setsockopt(zmq.SUBSCRIBE,b'');s.connect(f'tcp://127.0.0.1:{ports[name]}');sockets.append(s);receivers[name]=s
    root=Path(tempfile.mkdtemp(prefix='model_only_'+fault));master,slave=pty.openpty();out=(root/'stdout').open('w')
    cmd=[sys.executable,'-m','gear_sonic.scripts.run_dit4dit_model','--execute','--task-prompt','TEST',
        '--limits','gear_sonic/configs/dex1_limits.g1_20261013.json','--log-dir',str(root/'run'),'--duration','18']
    for name,port in ports.items():cmd+=['--'+name+'-port',str(port)]
    proc=subprocess.Popen(cmd,stdin=slave,stdout=out,stderr=out);os.close(slave)
    cache=LowStateCache();jpeg=ImageMessageSchema({},dict(ego_view=np.zeros((32,32,3),np.uint8))).serialize()['images']
    start=time.monotonic();seq=0;phase=0;ready=False;first_pose=None;token_ready=False;bottom='WAIT_FOR_CONTROL';tokens=0;fault_at=None;stop_at=None;requests_after=0
    try:
        while time.monotonic()-start<17:
            now=time.monotonic();seq+=1
            if proc.poll() is not None:raise AssertionError((root/'stdout').read_text())
            camera.send(msgpack.packb(dict(timestamps=dict(ego_view=time.time()),images=jpeg),use_bin_type=True))
            if not (fault_at and fault=='body'):cache.accept(seq,np.arange(29)*.01,now)
            body.send(TOPIC+msgpack.packb(cache.packet('test',seq,now),use_bin_type=True))
            sides={s:dict(q=1.,feedback_time=now,feedback_valid=not(fault_at and fault=='dex')) for s in ('left','right')}
            dex.send(b'dex1_telemetry '+msgpack.packb(dict(time=now,phase=4 if ready else 1,sides=sides),use_bin_type=True))
            if first_pose and now-first_pose>.4:token_ready=True
            state.send(b'dex1_control_status '+msgpack.packb(dict(time=now,seq=seq,state=bottom,stop=bottom=='STOP',token_ready=token_ready),use_bin_type=True))
            if policy.poll(0):
                req=policy.recv_json()
                if fault_at and now-fault_at>.5:requests_after+=1
                arr=np.ones((50,66)).tolist()
                if fault_at and fault=='invalid':arr=[[float('nan')]*66]*50
                if not (fault_at and fault=='timeout'):
                    policy.send_json(dict(ok=True,request_id=req['request_id'],generation=req['generation'],observed_at=req['observed_at'],actions=arr,normalized=arr))
            while receivers['target'].poll(0):
                data=msgpack.unpackb(receivers['target'].recv()[len(b'dex1_target '):],raw=False)
                if data.get('mode')=='arm':ready=True
            while receivers['action'].poll(0):
                raw=receivers['action'].recv();c=command_fields(raw)
                if c:
                    if c['start']:
                        assert token_ready,'start before token ACK';bottom='CONTROL'
                    if c['stop']:
                        if stop_at and now-stop_at>1:raise AssertionError('STOP repeats after ACK/restart')
                        bottom='STOP'
                elif raw.startswith(b'pose'):
                    tokens+=1
                    if first_pose is None:first_pose=now
                    if stop_at and now-stop_at>.15:raise AssertionError('model frames after stop')
            events=root/'run/events.jsonl';rows=[]
            if events.exists():
                for line in events.read_text().splitlines():
                    try:rows.append(json.loads(line))
                    except json.JSONDecodeError:pass
            if phase==0 and any(r['kind']=='prediction' for r in rows):
                assert tokens==0;os.write(master,b'arm\n');phase=1
            elif phase==1 and ready:
                # Give READY telemetry time to reach the gateway before manual model.
                phase=2;arm_ready_at=now
            elif phase==2 and now-arm_ready_at>.3:
                os.write(master,b'model\n');phase=3
            elif phase==3 and any(r['kind']=='mode' and r['mode']=='model' for r in rows):
                fault_at=now;phase=4
                if fault=='o':bottom='STOP'
            elif phase==4 and any(r['kind']=='mode' and r['mode']=='stop' for r in rows):
                stop_at=now;phase=5;os.write(master,b'model\n')
            elif phase==5 and now-stop_at>.4:
                bottom='WAIT_FOR_CONTROL';phase=6
            elif phase==6 and now-stop_at>1.2:
                assert any(r['kind']=='command_rejected' for r in rows)
                if fault=='body':assert requests_after==0,requests_after
                assert tokens>0
                print('PASS',fault,'prefill/model frames',tokens,'log',root,flush=True);return
            time.sleep(.01)
        raise AssertionError(str(root)+'\n'+(root/'stdout').read_text()[-4000:])
    finally:
        os.write(master,b'quit\n')
        try:proc.wait(timeout=4)
        except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=3)
        os.close(master);out.close()
        for s in sockets:s.close()
        ctx.term()
if __name__=='__main__':
    for f in ('o','body','dex','invalid','timeout'):scenario(f)
