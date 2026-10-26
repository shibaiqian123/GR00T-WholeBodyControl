"""Dex1 continuous proportional control with startup and endpoint braking. No DDS, robot connection or import-time side effects."""
from dataclasses import dataclass
import math

SIDES = ('left', 'right')

@dataclass(frozen=True)
class Limits:
    closed: float
    opened: float
    open_margin: float = 0.0

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.closed, self.opened)) or self.closed == self.opened:
            raise ValueError('Each side needs distinct, finite calibrated closed/open q endpoints')
        if not math.isfinite(self.open_margin) or not 0 <= self.open_margin < abs(self.opened-self.closed):
            raise ValueError('Opening margin must be finite, nonnegative and smaller than calibrated travel')

    @property
    def command_open(self):
        return self.opened + math.copysign(self.open_margin, self.closed-self.opened)

    def contains(self, q):
        # Feedback is measured, not a command: allow small encoder noise and
        # endpoint tracking overshoot (at most 0.02 rad / 1% of travel).
        # Every command, including startup/resume/hold, stays strictly clamped.
        tolerance = min(0.02, 0.01 * abs(self.opened-self.closed))
        return math.isfinite(q) and min(self.closed, self.opened) - tolerance <= q <= max(self.closed, self.opened) + tolerance

    def clamp(self, q):
        return max(min(self.closed, self.opened), min(max(self.closed, self.opened), q))

def endpoint_braked_target(limit, previous, requested, measured_q, measured_dq, dt,
                           zone=.05, max_speed=.15, min_speed=.02):
    """Shape only outward motion in the final 5% of travel.

    Normalized speeds are fractions of travel per second. A step from the
    interior first stops at the zone boundary; measured position AND speed
    must settle there before progressing. Reversing away from an end is direct.
    No gains, torque limits, watchdogs or calibrated feedback bounds change.
    """
    span = limit.command_open-limit.closed
    old = max(0., min(1., (previous-limit.closed)/span))
    goal = max(0., min(1., (requested-limit.closed)/span))
    actual = (measured_q-limit.closed)/span
    velocity = measured_dq/span
    high = goal > 1-zone and goal > old
    low = goal < zone and goal < old
    if not high and not low:
        return limit.clamp(requested), False
    # Mirror the closed end so the same outward logic handles both directions.
    if low:
        old, goal, actual, velocity = 1-old, 1-goal, 1-actual, -velocity
    boundary=1-zone
    if old < boundary:
        shaped=boundary
    elif abs(actual-old) > .02 or abs(velocity) > max_speed:
        shaped=old  # Settle after an interior step; do not pre-run the end ramp.
    else:
        speed=min(max_speed,max(min_speed,2.*max(0.,1-old)))
        shaped=min(goal,old+speed*min(.04,max(0.,dt)))
    if low:shaped=1-shaped
    return limit.clamp(limit.closed+shaped*span), True

class ObstructionHold:
    """Position-based contact heuristic; not a motor fault or torque limiter."""
    def __init__(self, duration=1.0, error=.15, speed=.05, travel=.02):
        if not all(math.isfinite(x) and x > 0 for x in (duration, error, speed, travel)):
            raise ValueError('Obstruction thresholds must be positive finite numbers')
        self.duration, self.error, self.speed, self.travel = duration, error, speed, travel
        self.reset()

    def reset(self):
        self.start = self.anchor = self.last_time = self.last_rx = None
        self.hold_q = self.release_trigger = None
        self.status = {'state': 'idle', 'elapsed': 0.0}

    def update(self, limit, now, feedback, trigger, target):
        q, rx = feedback[:2]
        sign = math.copysign(1., limit.closed-limit.command_open)
        if self.hold_q is not None:
            if trigger > self.release_trigger + 1e-9:
                self.status = {'state': 'holding', 'hold_q': self.hold_q,
                               'release_trigger': self.release_trigger}
                return self.hold_q
            self.reset()
            self.status = {'state': 'released', 'elapsed': 0.0}
            return target
        # Require new feedback at each counted step and no scheduling gap >100 ms.
        continuous = (self.last_time is not None and 0 < now-self.last_time <= .1
                      and self.last_rx is not None and rx > self.last_rx)
        self.last_time, self.last_rx = now, rx
        dq = feedback[2] if len(feedback) >= 3 else math.inf
        requested = limit.command_open + trigger*(limit.closed-limit.command_open)
        blocked = ((target-q)*sign >= self.error and (requested-q)*sign >= self.error
                   and abs(dq) <= self.speed)
        if not blocked:
            self.start = self.anchor = None
        elif not continuous or self.start is None or abs(q-self.anchor) > self.travel:
            self.start, self.anchor = now, q
        elapsed = 0. if self.start is None else now-self.start
        self.status = {'state': 'candidate' if blocked else 'idle', 'elapsed': elapsed}
        if blocked and elapsed + 1e-9 >= self.duration:
            self.hold_q = limit.clamp(q)
            self.release_trigger = max(0., min(1., (self.hold_q-limit.command_open) /
                                                     (limit.closed-limit.command_open)))
            self.status = {'state': 'holding', 'hold_q': self.hold_q,
                           'release_trigger': self.release_trigger}
            return self.hold_q
        return target

class RampController:
    def __init__(self, limits, travel_time=2.0, timeout=.25, *, obstruction_hold=False,
                 obstruction_timeout=1.0):
        if set(limits) != set(SIDES):
            raise ValueError('Both left and right limits are required')
        if not math.isfinite(travel_time) or travel_time <= 0 or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Travel time and timeout must be positive finite numbers')
        self.obstruction_enabled = obstruction_hold
        self.obstruction = {s: ObstructionHold(obstruction_timeout) for s in SIDES}
        self.limits = limits
        self.travel_time = travel_time
        self.timeout = timeout
        self.phase = 'WAIT_INPUTS'
        self.reason = ''
        self.first_fault = None
        self.feedback_status = {}
        self.target = {}
        self.requested = {}
        self.motion = dict.fromkeys(SIDES, "holding")
        self.endpoint_braking = dict.fromkeys(SIDES, False)
        self.hold_captured = set()
        self.last_time = None

    @property
    def obstruction_status(self):
        return {s: dict(v.status) for s, v in self.obstruction.items()} if self.obstruction_enabled else None

    def _fresh(self, timestamp, now):
        return math.isfinite(timestamp) and 0 <= now - timestamp <= self.timeout

    def update(self, now, measured, trigger, heartbeat, resume=False):
        """measured: side -> (q, local_rx_time, dq); trigger/heartbeat are validated dictionaries.

        Returns only sides with fresh valid feedback. STOP/invalid/stale input is
        latched until explicit resume with both triggers released and live controller.
        """
        dt = 0.0 if self.last_time is None else min(.04, max(0., now-self.last_time))
        self.last_time = now
        good = {s: (s in measured and self._fresh(measured[s][1], now)
                    and self.limits[s].contains(measured[s][0])
                    and (len(measured[s]) < 3 or math.isfinite(measured[s][2]))) for s in SIDES}
        self.feedback_status = {}
        for side in SIDES:
            limit = self.limits[side]
            if side not in measured:
                self.feedback_status[side] = {"error": "missing"}
                continue
            q, rx = measured[side][:2]
            dq = measured[side][2] if len(measured[side]) >= 3 else 0.
            error = ("nonfinite_position" if not math.isfinite(q) else
                     "nonfinite_velocity" if not math.isfinite(dq) else
                     "stale_or_invalid_time" if not self._fresh(rx, now) else
                     "outside_calibrated_range" if not limit.contains(q) else "")
            self.feedback_status[side] = {
                "error": error, "q": q, "dq": dq, "age_ms": (now-rx)*1000.,
                "min_q": min(limit.closed, limit.opened), "max_q": max(limit.closed, limit.opened),
                "feedback_tolerance_rad": min(.02, .01*abs(limit.opened-limit.closed)),
                "command_open_rad": limit.command_open, "open_margin_rad": limit.open_margin}
        tf = bool(trigger and trigger.get('valid') is True and self._fresh(trigger.get('sample_time', -math.inf), now)
                  and self._fresh(trigger.get('time', -math.inf), now))
        if tf:
            tf = all(isinstance(trigger.get(s), (int, float)) and math.isfinite(trigger[s]) and 0 <= trigger[s] <= 1 for s in SIDES)
        hf = bool(heartbeat and self._fresh(heartbeat.get('time', -math.inf), now)
                  and heartbeat.get('state') in ('INIT','WAIT_FOR_CONTROL','CONTROL')
                  and heartbeat.get('stop') is False)
        healthy = all(good.values()) and tf and hf and not trigger.get('stop', False)
        if not healthy:
            for guard in self.obstruction.values():
                guard.reset()
            failures = [f"{side}: {info['error']}" for side, info in self.feedback_status.items() if info['error']]
            reason = ("feedback " + "; ".join(failures) if failures else
                      'trigger invalid/stale/stop' if not tf or trigger.get('stop', False) else 'controller stopped/stale')
            if self.phase != 'FAULT_HOLD':
                self.reason = reason
            if self.phase != 'WAIT_INPUTS' and self.phase != 'FAULT_HOLD':
                self.first_fault = {"time": now, "reason": reason,
                    "feedback": {side: dict(info) for side, info in self.feedback_status.items()},
                    "previous_target": dict(self.target),
                    "trigger": dict(trigger) if trigger else None,
                    "heartbeat": dict(heartbeat) if heartbeat else None}
                self.phase = 'FAULT_HOLD'; self.hold_captured.clear()
        if self.phase == 'WAIT_INPUTS':
            if not healthy:
                return {}
            self.target = {s: self.limits[s].clamp(measured[s][0]) for s in SIDES}
            self.phase = 'OPENING'
            dt = 0.0
        if self.phase == 'FAULT_HOLD':
            for s in SIDES:
                if good[s] and s not in self.hold_captured:
                    self.target[s] = self.limits[s].clamp(measured[s][0]); self.hold_captured.add(s)
                elif not good[s]:
                    self.hold_captured.discard(s)
            if resume and healthy and all(trigger[s] <= .10 for s in SIDES):
                self.target = {s: self.limits[s].clamp(measured[s][0]) for s in SIDES}
                self.motion = dict.fromkeys(SIDES, "holding")
                self.phase = 'READY'; self.reason = ''; self.first_fault = None
                return dict(self.target)
            else:
                return {s:self.target[s] for s in SIDES if good[s] and s in self.hold_captured}
        self.reason = ''
        if self.phase == 'RELEASE_TO_ARM':
            if all(trigger[s] <= .10 for s in SIDES):
                self.phase = 'READY'
            return dict(self.target)
        opening = self.phase == 'OPENING'
        for s in SIDES:
            lim = self.limits[s]
            # Absolute position mapping: holding a partial trigger holds that
            # opening, rather than integrating a continuous closing command.
            destination = lim.command_open if opening else lim.command_open + trigger[s] * (lim.closed-lim.command_open)
            self.requested[s] = destination
            step = abs(lim.command_open-lim.closed) * dt / self.travel_time
            delta = destination-self.target[s]
            self.motion[s] = ("holding" if abs(delta) <= 1e-9 else
                              "closing" if delta * (lim.closed-lim.opened) > 0 else "opening")
            if opening:
                self.target[s] = destination if abs(delta) <= step + 1e-9 else lim.clamp(self.target[s] + max(-step,min(step,delta)))
            else:
                dq = measured[s][2] if len(measured[s]) >= 3 else 0.
                self.target[s], self.endpoint_braking[s] = endpoint_braked_target(
                    lim, self.target[s], destination, measured[s][0], dq, dt)
                if self.obstruction_enabled:
                    self.target[s] = self.obstruction[s].update(
                        lim, now, measured[s], trigger[s], self.target[s])
                    if self.obstruction[s].hold_q is not None:
                        self.motion[s] = "obstruction_hold"
                        self.endpoint_braking[s] = False
        if opening and all(abs(self.target[s]-self.limits[s].command_open)<1e-9 for s in SIDES):
            # Wait for the physical grippers too: an obstruction must not arm closing.
            if all(abs(measured[s][0]-self.limits[s].command_open) <= .02*abs(self.limits[s].opened-self.limits[s].closed) for s in SIDES):
                self.phase = 'RELEASE_TO_ARM'
        return dict(self.target)


class TriggerFrameClock:
    """Freshness of SDK input frames, independent of the body processing worker.

    Observing a cached timestamp for the first time does NOT make it fresh.
    At least one forward advance is required. Re-reading never refreshes age.
    """
    def __init__(self):
        self.last_stamp = None
        self.last_advance = None

    def observe(self, stamp_ns, now):
        if not isinstance(stamp_ns, (int, float)) or not math.isfinite(stamp_ns) or stamp_ns <= 0:
            self.last_stamp = None
            self.last_advance = None
            return 0.0, False
        if self.last_stamp is None or stamp_ns < self.last_stamp:
            self.last_advance = None
        elif stamp_ns > self.last_stamp:
            self.last_advance = now
        self.last_stamp = stamp_ns
        fresh = self.last_advance is not None and 0 <= now-self.last_advance <= .25
        return self.last_advance if self.last_advance is not None else 0.0, fresh
