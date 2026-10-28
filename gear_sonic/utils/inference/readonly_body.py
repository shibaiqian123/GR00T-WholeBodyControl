"""Same-host read-only G1 state contract. No robot publishers or control imports."""
import math
import numpy as np

TOPIC = b'dit4dit_body_readonly '
ISAACLAB_TO_MUJOCO = (0,3,6,9,13,17,1,4,7,10,14,18,2,5,8,11,15,19,21,23,25,27,12,16,20,22,24,26,28)
MUJOCO_TO_ISAACLAB = (0,6,12,1,7,13,2,8,14,3,9,15,22,4,10,16,23,5,11,17,24,18,25,19,26,20,27,21,28)


def dds_to_body_q(q):
    """C++ logger's reorder/subtract then publisher's inverse/add cancel exactly.

    See GatherRobotStateToLogger and zmq_output_handler.hpp (body_q).
    Values here are absolute motor radians, NOT centered policy observations.
    """
    q = np.asarray(q, dtype=np.float64)
    if q.shape != (29,) or not np.isfinite(q).all():
        raise ValueError('Expected 29 finite absolute motor positions')
    return q.copy()


def advancing_tick(previous, current):
    if type(current) is not int or not 0 <= current < 2**32:
        return False
    return previous is None or 0 < ((current-previous) % 2**32) < 2**31


class LowStateCache:
    def __init__(self):
        self.tick = None
        self.stamp = None
        self.q = None
        self.error = 'no_dds_feedback'
        self.duplicates = 0
        self.last_rejected = None

    def accept(self, tick, q, now):
        try:
            q = dds_to_body_q(q)
            if type(tick) is int and tick == self.tick:
                self.last_rejected = dict(reason='same_tick',tick=tick,position_delta=float(np.max(np.abs(q-self.q))))
                if not math.isfinite(now) or now < self.stamp:
                    raise ValueError('invalid_receive_time')
                self.duplicates += 1
                # Ignore identical retransmissions. Never renew stamp or clear a fault.
                return False
            if not advancing_tick(self.tick, tick):
                raise ValueError('nonadvancing_tick:previous='+str(self.tick)+':incoming='+str(tick))
            if not math.isfinite(now) or (self.stamp is not None and now <= self.stamp):
                raise ValueError('invalid_receive_time')
            self.tick, self.q, self.stamp, self.error = tick, q, now, ''
            return True
        except (ValueError, TypeError) as exc:
            self.error = str(exc)
            return False

    def packet(self, session, seq, now):
        valid = (self.q is not None and 0 <= now-self.stamp <= .25 and not self.error)
        return dict(version=1,session=session,seq=seq,source_tick=self.tick,
                    time=now,sample_time=self.stamp,valid=valid,duplicate_samples=self.duplicates,last_rejected=self.last_rejected,
                    body_q=self.q.tolist() if valid else None,
                    error='' if valid else self.error or 'dds_feedback_expired')


class PacketCache:
    def __init__(self):
        self.packet = None
        self.error = 'no_body_packet'
        self.last_valid = None

    def accept(self, p, now):
        try:
            if p['version'] != 1 or type(p['session']) is not str or not p['session']:
                raise ValueError('schema')
            if type(p['seq']) is not int or p['seq'] < 0 or type(p['valid']) is not bool:
                raise ValueError('flags/sequence')
            if not 0 <= now-p['time'] <= .25:
                raise ValueError('packet_expired')
            if not p['valid']:
                self.error = p.get('error','invalid_body'); self.packet = None
                return False
            if not 0 <= now-p['sample_time'] <= .25:
                raise ValueError('sample_expired')
            if not advancing_tick(None,p['source_tick']):
                raise ValueError('tick')
            dds_to_body_q(p['body_q'])
            old = self.last_valid
            if old and old['session']==p['session']:
                if p['seq'] <= old['seq'] or p['time'] <= old['time']:
                    return False
                # Republishing a cached sample cannot renew its receive time.
                if p['source_tick']==old['source_tick']:
                    if p['sample_time'] != old['sample_time']:
                        raise ValueError('repeated_tick_renewed_timestamp')
                elif not advancing_tick(old['source_tick'],p['source_tick']) or p['sample_time']<=old['sample_time']:
                    raise ValueError('source_not_advancing')
            self.packet = p; self.last_valid = p; self.error = ''; return True
        except (KeyError,TypeError,ValueError,OverflowError):
            self.packet = None; self.error = 'invalid_body_packet'; return False

    def current(self, now):
        if self.packet and 0 <= now-self.packet['sample_time'] <= .25:
            return self.packet
        self.error = self.error or 'body_sample_expired'
        return None


class ReadOnlyBodySubscriber:
    def __init__(self, port):
        import zmq
        self.context=zmq.Context();self.socket=self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.LINGER,0);self.socket.setsockopt(zmq.CONFLATE,1)
        self.socket.setsockopt(zmq.SUBSCRIBE,TOPIC)
        self.socket.connect(f'tcp://127.0.0.1:{port}')
        self.cache=PacketCache()
    def get_msg(self):
        import zmq, msgpack, time
        try:
            raw=self.socket.recv(zmq.NOBLOCK)
            self.cache.accept(msgpack.unpackb(raw[len(TOPIC):],raw=False),time.monotonic())
        except zmq.Again:
            pass
        except (ValueError,TypeError):
            self.cache.packet=None;self.cache.error='malformed_body_packet'
        return self.cache.current(time.monotonic())
    def close(self):
        self.socket.close();self.context.term()
