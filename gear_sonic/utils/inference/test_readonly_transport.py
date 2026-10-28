"""Real gateway, simulated sensors/model; no DDS, no robot action endpoints."""
import json, subprocess, sys, tempfile, time
from pathlib import Path
import msgpack
import numpy as np
import zmq
from gear_sonic.camera.sensor_server import ImageMessageSchema
from gear_sonic.utils.inference.readonly_body import LowStateCache, TOPIC


def main():
    ctx=zmq.Context(); sockets=[];ports={}
    for name in ('camera','body','dex','policy','action','target'):
        sock=ctx.socket(zmq.REP if name=='policy' else zmq.PUB)
        sock.setsockopt(zmq.LINGER,0)
        ports[name]=sock.bind_to_random_port('tcp://127.0.0.1',min_port=16000,max_port=30000)
        sockets.append(sock)
    camera,body,dex,policy,action,target=sockets
    # Keep action/target ports occupied: a gateway accidentally binding them fails.
    root=Path(tempfile.mkdtemp(prefix='readonly_transport_'));out=(root/'stdout').open('w')
    cmd=[sys.executable,'-m','gear_sonic.scripts.run_dit4dit_gateway',
         '--task-prompt','TEST','--limits','gear_sonic/configs/dex1_limits.g1_20261013.json',
         '--log-dir',str(root/'run'),'--duration','12']
    for flag,name in [('camera','camera'),('readonly-body','body'),('dex-telemetry','dex'),('policy','policy'),('action','action'),('target','target')]:
        cmd.extend(['--'+flag+'-port',str(ports[name])])
    proc=subprocess.Popen(cmd,stdin=subprocess.DEVNULL,stdout=out,stderr=out)
    cache=LowStateCache();first=None;count=0;late=0;seq=0
    jpeg=ImageMessageSchema({},dict(ego_view=np.zeros((32,32,3),dtype=np.uint8))).serialize()['images']
    start=time.monotonic()
    try:
        while proc.poll() is None and time.monotonic()-start<20:
            now=time.monotonic();seq+=1
            camera.send(msgpack.packb(dict(timestamps=dict(ego_view=time.time()),images=jpeg),use_bin_type=True))
            # After first request + 1 sec, publish cached samples without renewing DDS time.
            if first is None or now-first<1:cache.accept(seq,np.arange(29)*.01,now)
            body.send(TOPIC+msgpack.packb(cache.packet('simulation',seq,now),use_bin_type=True))
            sides={s:dict(q=.1,feedback_time=now,feedback_valid=True) for s in ('left','right')}
            dex.send(b'dex1_telemetry '+msgpack.packb(dict(time=now,phase=0,sides=sides),use_bin_type=True))
            if policy.poll(0):
                req=policy.recv_json();count+=1
                assert np.asarray(req['state']).shape==(31,)
                if first is None:first=now
                if now-first>1.5:late+=1
                actions=np.zeros((50,66)).tolist()
                policy.send_json(dict(ok=True,request_id=req['request_id'],generation=req['generation'],
                   observed_at=req['observed_at'],actions=actions,normalized=actions,inference_seconds=.01))
            time.sleep(.01)
        proc.wait(timeout=4)
        assert proc.returncode==0,(root/'stdout').read_text()
        assert count>=2,(count,(root/'stdout').read_text())
        assert late==0,late
        rows=[json.loads(x) for x in (root/'run/events.jsonl').read_text().splitlines()]
        assert any(r['kind']=='health' and not r['sensors_ok'] and r['body_error'] for r in rows)
        assert not any(r['kind']=='frame' for r in rows)
        print('PASS readonly transport, requests',count,'requests after expiration',late,'log',root)
    finally:
        if proc.poll() is None:proc.terminate();proc.wait(timeout=5)
        out.close()
        for s in sockets:s.close()
        ctx.term()

if __name__=='__main__':main()
