"""Client owns observation timestamps; server clocks never authorize actions."""
import queue
import time


def bind_reply(req, reply, sent, received):
    if not isinstance(reply, dict) or reply.get('request_id') != req['request_id'] or reply.get('generation') != req['generation']:
        raise ValueError('Mismatched inference response')
    reply = dict(reply)
    reply['observed_at'] = req['observed_at']
    reply['preparation_seconds'] = req.get('preparation_seconds',0.)
    reply['roundtrip_seconds'] = received - sent
    reply['observation_age_seconds'] = received - req['observed_at']
    if not 0 <= reply['observation_age_seconds'] < .95:
        reply['ok'] = False
        reply['error'] = 'Prediction expired on client clock'
    return reply


def prepare_request(request):
    """Worker-only CPU/disk work. Archive raw input; transmit training-sized RGB."""
    if '_input' not in request:
        return request
    import base64
    import io
    import numpy as np
    from PIL import Image
    started=time.monotonic()
    req=dict(request)
    raw=req.pop('_input')
    path=req.pop('_input_path')
    # This is the same RGB/BILINEAR preprocessing used by the policy server.
    image=Image.fromarray(raw['image']).convert('RGB').resize((224,224),Image.Resampling.BILINEAR)
    buf=io.BytesIO()
    image.save(buf,format='PNG')
    req['image_png']=base64.b64encode(buf.getvalue()).decode('ascii')
    # Preserve original resolution and all timestamps; avoid costly ZIP compression.
    np.savez(path,**raw)
    req['preparation_seconds']=time.monotonic()-started
    return req


def worker(endpoint, requests, replies, done):
    import zmq
    ctx = zmq.Context()
    try:
        while not done.is_set():
            try:
                req = requests.get(timeout=.1)
            except queue.Empty:
                continue
            sock = ctx.socket(zmq.REQ)
            sock.setsockopt(zmq.LINGER, 0)
            sock.setsockopt(zmq.RECONNECT_IVL, 1000)
            sock.connect(endpoint)
            try:
                if done.is_set():break
                req = prepare_request(req)
                sent = time.monotonic()
                if not 0 <= sent - req['observed_at'] <= .5:
                    raise ValueError('Observation expired before send')
                sock.send_json(req)
                if not sock.poll(2000):
                    raise TimeoutError('Inference service timeout')
                reply = bind_reply(req, sock.recv_json(), sent, time.monotonic())
                replies.put(reply)
            except Exception as exc:
                replies.put(dict(ok=False,error=str(exc),generation=req['generation'],request_id=req['request_id']))
            finally:
                sock.close()
    finally:
        ctx.term()
