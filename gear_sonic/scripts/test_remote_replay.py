"""Recorded observations only; creates no robot command sockets."""
import base64,io,json,subprocess,time
from pathlib import Path
import numpy as np
from PIL import Image
import zmq
from gear_sonic.utils.inference.remote_policy_client import bind_reply
from gear_sonic.utils.inference.dit4dit_control import validate_actions
files=sorted(Path('outputs').glob('model_only_*/gateway/input_*.npz'))[-10:]
assert files,'No recorded observations'
child=subprocess.Popen(['ssh','-N','-T','-o','LogLevel=ERROR','-o','BatchMode=yes','-o','ExitOnForwardFailure=yes','-L','127.0.0.1:15560:127.0.0.1:5560','2025slx@10.246.1.32'])
ctx=zmq.Context();rows=[]
try:
    time.sleep(2)
    assert child.poll() is None,'SSH tunnel failed'
    deadline=time.monotonic()+600
    while True:
        probe=ctx.socket(zmq.REQ);probe.setsockopt(zmq.LINGER,0);probe.setsockopt(zmq.RECONNECT_IVL,1000);probe.connect('tcp://127.0.0.1:15560')
        probe.send_json({'op':'ping'})
        available=probe.poll(5000)
        if available:
            assert probe.recv_json()['ok'];probe.close();break
        probe.close()
        if time.monotonic()>deadline:raise TimeoutError('Model service did not become ready')
    print('Remote policy READY through SSH',flush=True)
    for i,f in enumerate(files):
        d=np.load(f);buf=io.BytesIO();Image.fromarray(d['image']).save(buf,format='PNG')
        observed=time.monotonic()
        req=dict(op='infer',request_id=i,generation=0,observed_at=observed,state=d['state'].tolist(),lang='Pick up the cup on the left with your left gripper. Then pick up the red block on the right with your right gripper and place it into the cup held in your left gripper. Finally, place the cup, with the red block inside, back on the table using your left gripper.',image_png=base64.b64encode(buf.getvalue()).decode())
        s=ctx.socket(zmq.REQ);s.setsockopt(zmq.LINGER,0);s.connect('tcp://127.0.0.1:15560')
        start=time.monotonic();s.send_json(req)
        assert s.poll(120000),'Server replay timeout'
        raw=s.recv_json();end=time.monotonic();s.close()
        assert raw['ok'],raw
        validate_actions(raw['actions']);validate_actions(raw['normalized'])
        bound=bind_reply(req,raw,start,end)
        rows.append(dict(file=str(f),rtt_seconds=end-start,inference_seconds=raw['inference_seconds'],client_fresh=bound['ok']))
        print(rows[-1],flush=True)
    report=dict(kind='recorded_observation_replay_not_live_robot',rows=rows,median_rtt=float(np.median([r['rtt_seconds'] for r in rows[1:]])))
    Path('outputs/remote_policy_replay.json').write_text(json.dumps(report,indent=2))
finally:
    child.terminate();child.wait(timeout=5);ctx.term()
