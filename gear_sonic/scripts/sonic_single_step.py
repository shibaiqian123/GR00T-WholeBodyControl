"""Explicit one-shot request client. Default status only; never starts robot processes."""
import argparse
import json
import time
import uuid
from pathlib import Path
import numpy as np
import zmq

def rpc(ctx, request):
    s=ctx.socket(zmq.REQ);s.setsockopt(zmq.LINGER,0)
    s.connect('tcp://127.0.0.1:15558')
    try:
        s.send_json(request)
        if not s.poll(2000):raise TimeoutError('No reply; execution outcome unknown. Query the same request ID; do not resend with a new ID.')
        reply=s.recv_json()
        if not reply.get('ok'):raise RuntimeError(reply.get('error','request rejected'))
        return reply
    finally:s.close()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true',help='Explicitly request one motor frame, then PLANNER')
    p.add_argument('--token',type=Path)
    p.add_argument('--output',type=Path,help='New directory for execute; existing directory for receipt polling')
    a=p.parse_args()
    if a.execute and (not a.token or not a.output):p.error('--execute requires --token and --output')
    with zmq.Context() as ctx:
        if a.execute:
            token=np.load(a.token,allow_pickle=False)
            if token.shape!=(64,) or not np.isfinite(token).all() or not np.isfinite(token.astype(np.float32)).all():raise ValueError('64 finite float32-compatible values required')
            status=rpc(ctx,{'op':'status'})
            if not status['ready']:raise RuntimeError('Active fresh PLANNER required. This tool will not start or switch it.')
            request=dict(op='execute_once',session=status['session'],request_id=uuid.uuid4().hex,token=token.tolist(),execute=True)
            a.output.mkdir(parents=True,exist_ok=False)
            (a.output/'request.json').write_text(json.dumps(request,indent=2))
            reply=rpc(ctx,request)
        elif a.output:
            request=json.loads((a.output/'request.json').read_text())
            status=rpc(ctx,{'op':'status'})
            if status['session']!=request['session']:raise RuntimeError('Bottom session changed; old result cannot be recovered here. Do not automatically execute again.')
            reply=rpc(ctx,dict(op='status',request_id=request['request_id']))
        else:
            print(json.dumps(rpc(ctx,{'op':'status'}),indent=2));return
        deadline=time.monotonic()+5
        while True:
            (a.output/'receipt.json').write_text(json.dumps(reply,indent=2,allow_nan=False))
            with (a.output/'receipt_history.jsonl').open('a') as f:f.write(json.dumps(dict(client_time=time.monotonic(),receipt=reply),allow_nan=False)+'\n')
            if reply.get('phase') in ('complete','failed'):break
            if time.monotonic()>deadline:raise TimeoutError('Still pending. Poll this output directory; do not submit another execution.')
            time.sleep(.05)
            reply=rpc(ctx,dict(op='status',request_id=request['request_id']))
        print(json.dumps({k:v for k,v in reply.items() if k not in ('decoder_input','token','raw_action','mapping','scale','default_angles')},indent=2))
        if reply.get('phase')!='complete':raise SystemExit(1)
        print('DDS send and PLANNER return confirmed; physical target reached is NOT confirmed.')
if __name__=='__main__':main()
