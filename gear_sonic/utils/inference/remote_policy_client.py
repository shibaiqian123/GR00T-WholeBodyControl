"""Client owns observation timestamps; server clocks never authorize actions."""
import queue
import time


def bind_reply(req, reply, sent, received):
    if not isinstance(reply, dict) or reply.get('request_id') != req['request_id'] or reply.get('generation') != req['generation']:
        raise ValueError('Mismatched inference response')
    reply = dict(reply)
    reply['observed_at'] = req['observed_at']
    reply['roundtrip_seconds'] = received - sent
    reply['observation_age_seconds'] = received - req['observed_at']
    if not 0 <= reply['observation_age_seconds'] < .95:
        reply['ok'] = False
        reply['error'] = 'Prediction expired on client clock'
    return reply


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
