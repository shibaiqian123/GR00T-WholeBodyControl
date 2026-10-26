"""Opt-in ViBe preparation handshake; no hardware or model dependency."""
import json,time,uuid

def sample_grid(previous,current,target,step=20_000_000):
    """Only interpolate inside two real source samples; never extrapolate."""
    if current<=previous:raise ValueError("body timestamp did not advance")
    if current-previous>250_000_000:raise ValueError("body sample gap exceeds 250 ms")
    if target<previous:raise ValueError("resampling target fell outside source interval")
    return list(range(target,current+1,step))

def status_allows(status,request_id,session,now):
    if not isinstance(status,dict) or not session:return False
    try:
        return (status.get("prepare_id")==request_id and status.get("session")==session
                and status.get("ready") is True
                and 0<=now-float(status.get("sent_monotonic",-1))<=.15)
    except (TypeError,ValueError):return False

class PoseGate:
    def __init__(self,context,publisher):
        import zmq
        self.publisher=publisher
        self.sub=context.socket(zmq.SUB)
        self.sub.setsockopt(zmq.SUBSCRIBE,b"vibe_ready ")
        self.sub.setsockopt(zmq.LINGER,0)
        self.sub.connect("tcp://127.0.0.1:5583")
        self.request_id=None;self.status=None;self.last_send=0.;self.last_report=0.
    def begin(self):
        self.request_id=uuid.uuid4().hex;self.started=time.monotonic()
        self.status=None;self.last_send=0.;self.last_report=0.
    def cancel(self):
        self.request_id=None;self.status=None
    def poll(self,session):
        import zmq
        while self.sub.poll(0):
            try:self.status=json.loads(self.sub.recv()[len(b"vibe_ready "):])
            except (ValueError,KeyError):self.status=None
        now=time.monotonic()
        if now-self.last_send>=.05:
            self.publisher.send(b"vibe_prepare "+json.dumps({
                "prepare_id":self.request_id,"session":session}).encode())
            self.last_send=now
        return status_allows(self.status,self.request_id,session,now)
    def close(self):self.sub.close()
