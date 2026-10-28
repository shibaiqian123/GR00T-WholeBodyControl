"""Pure control arbitration. No robot/network side effects on import."""
import json
import math
import numpy as np

BODY_INDICES = list(range(22)) + list(range(29, 36))


def fresh(stamp, now, limit=.25):
    return isinstance(stamp, (int, float)) and math.isfinite(stamp) and 0 <= now-stamp <= limit


def sonic_state(whole_q, grippers):
    q = np.asarray(whole_q, dtype=np.float64)
    g = np.asarray(grippers, dtype=np.float64)
    if q.shape != (43,) or g.shape != (2,):
        raise ValueError('Expected collected whole_q[43] and measured Dex1[2]')
    s = np.concatenate((q[BODY_INDICES], g))
    if not np.isfinite(s).all():
        raise ValueError('Nonfinite robot state')
    return s


def normalize_state(state, stats):
    state = np.asarray(state, dtype=np.float32).reshape(1, 31)
    lo, hi = (np.asarray(stats[k]) for k in ('min', 'max'))
    if lo.shape != (31,) or hi.shape != (31,):
        raise ValueError('Wrong state statistics')
    out = np.zeros_like(state)
    mask = hi != lo
    out[:, mask] = 2*(state[:, mask]-lo[mask])/(hi[mask]-lo[mask])-1
    if not np.isfinite(out).all():
        raise ValueError('Invalid normalized state')
    return out


def validate_actions(value):
    a = np.asarray(value, dtype=np.float32)
    if a.shape != (50, 66) or not np.isfinite(a).all():
        raise ValueError('Actions must be finite [50,66]')
    return a


def command_fields(raw):
    """Parse only command packets from the existing SONIC wire protocol."""
    if not raw.startswith(b'command'):
        return None
    header = json.loads(raw[7:1287].rstrip(b'\0'))
    fields = header['fields']
    expected = [('start','u8',[1]), ('stop','u8',[1]), ('planner','u8',[1])]
    if [(f['name'], f['dtype'], f['shape']) for f in fields[:3]] != expected:
        raise ValueError('Unexpected command wire schema')
    if len(raw) < 1290 or any(v not in (0,1) for v in raw[1287:1290]):
        raise ValueError('Invalid command flags')
    return dict(zip(('start','stop','planner'), map(bool, raw[1287:1290])))


class Arbiter:
    def __init__(self):
        self.mode = 'readonly'
        self.generation = 0
        self.chunk = None
        self.reason = 'startup'
        self.bottom = None

    def clear(self):
        self.generation += 1
        self.chunk = None

    def stop(self, reason):
        if self.mode != 'stop':
            self.clear()
        self.mode, self.reason = 'stop', reason

    def status(self, packet, now):
        self.bottom = packet
        if packet and packet.get('stop') is True:
            self.stop('bottom_stop')
        elif self.mode in ('model','teleop') and not self.bottom_ok(now):
            self.stop('bottom_missing_or_stale')

    def bottom_ok(self, now):
        p = self.bottom or {}
        return (fresh(p.get('time'), now) and p.get('stop') is False
                and p.get('state') in ('WAIT_FOR_CONTROL','CONTROL'))

    def reset(self, now):
        if self.mode != 'stop':
            raise ValueError('reset is allowed only after STOP; use stop first')
        if not self.bottom_ok(now):
            raise ValueError('Restart bottom controller and wait for fresh non-STOP status')
        self.clear()
        self.mode, self.reason = 'readonly', 'manual_reset'

    def accept(self, result, now):
        if self.mode not in ('readonly','model','teleop') or result.get('generation') != self.generation:
            return False
        stamp = result.get('observed_at')
        if not fresh(stamp, now, 1.0) or now-stamp >= 1.0:
            return False
        actions = validate_actions(result['actions'])
        self.chunk = (stamp, actions)
        return True

    def select(self, mode, now, sensors_ok=False, pico_ok=False, dex_ready=False):
        if self.mode == 'stop':
            raise ValueError('Stop is latched; explicit reset required')
        if mode == 'readonly':
            if self.mode in ('model','teleop'):
                self.stop('manual_stop')
            return
        if not self.bottom_ok(now):
            raise ValueError('Bottom controller not ready')
        if mode == 'teleop':
            if not pico_ok:
                self.stop('pico_takeover_unavailable')
                raise ValueError('Fresh PICO planner and triggers required')
            self.clear()
            self.mode, self.reason = 'teleop', 'manual_takeover'
        elif mode == 'model':
            if not (sensors_ok and dex_ready and self.bottom['state']=='CONTROL'
                    and self.chunk and now-self.chunk[0] < .8):
                raise ValueError('Need live sensors, READY Dex1, CONTROL and recent prediction')
            self.mode, self.reason = 'model', 'manual_enable'
        else:
            raise ValueError('Unknown mode')

    def frame(self, now, sensors_ok):
        if self.mode != 'model':
            return None
        if not sensors_ok or not self.bottom_ok(now):
            self.stop('observation_or_status_stale')
            return None
        if not self.chunk:
            self.stop('no_prediction')
            return None
        idx = int((now-self.chunk[0])*50)
        if not 0 <= idx < 50:
            self.stop('action_expired')
            return None
        return self.chunk[1][idx].copy()


def target_to_trigger(packet, limits, now):
    """Retain RampController feedback/obstruction/endpoint protection."""
    if not (fresh(packet.get('time'), now) and fresh(packet.get('sample_time'), now)
            and packet.get('valid') is True and packet.get('stop') is False):
        raise ValueError('Invalid/stale Dex1 target')
    out = dict(packet)
    for s in ('left','right'):
        q = packet[s]
        lim = limits[s]
        if not isinstance(q,(int,float)) or not math.isfinite(q) or not lim.contains(q):
            raise ValueError('Dex1 target outside calibrated bounds')
        q = lim.clamp(q)
        out[s] = max(0., min(1., (q-lim.command_open)/(lim.closed-lim.command_open)))
    return out
