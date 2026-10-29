"""Synchronous scheduling without robot I/O. Live adapter continues SONIC on the last token.

A missing action means 'nothing scheduled', NOT 'robot is holding still'.
No output from this scheduler alone does not mean the robot is stationary.
"""
import math
from gear_sonic.utils.inference.dit4dit_control import fresh, validate_actions

class SynchronousPolicy:
    def __init__(self, segment_frames, defer_execution=False):
        if type(segment_frames) is not int or not 1 <= segment_frames <= 50:
            raise ValueError('segment_frames must be 1..50')
        self.defer_execution=defer_execution
        self.segment_frames=segment_frames
        self.phase='observe';self.generation=0;self.request_id=0
        self.pending=None;self.chunk=None;self.next_frame=0
        self.not_before=-math.inf;self.reason='startup';self.last_time=-math.inf
    def stop(self,reason):
        if self.phase!='stop':self.generation+=1
        self.phase='stop';self.reason=reason;self.pending=None;self.chunk=None
    def check(self,now,sensors_ok=True):
        if not isinstance(now,(int,float)) or not math.isfinite(now) or now<self.last_time:
            self.stop('invalid_clock');return False
        self.last_time=now
        if not sensors_ok:self.stop('invalid_observation')
        if self.phase=='infer' and now-self.pending['requested_at']>2:
            self.stop('inference_timeout')
        return self.phase!='stop'
    def request(self,observed_at,now,sensors_ok=True):
        if not self.check(now,sensors_ok) or self.phase!='observe':return None
        # A new cycle must use an observation actually acquired after the segment.
        if not fresh(observed_at,now,.5) or observed_at<self.not_before:return None
        self.pending=dict(request_id=self.request_id,generation=self.generation,
                          observed_at=observed_at,requested_at=now)
        self.request_id+=1;self.phase='infer'
        return dict(self.pending)
    def accept(self,result,now):
        if not self.check(now) or self.phase!='infer':return False
        if any(result.get(k)!=self.pending[k] for k in ('request_id','generation')):return False
        if result.get('ok') is not True:self.stop('inference_failed');return False
        # Use the original client observation timestamp, never the server clock.
        stamp=self.pending['observed_at']
        if not fresh(stamp,now,.5):self.stop('prediction_not_fresh');return False
        # Do not renew action age at execution start or extend the existing 1s horizon.
        if now-stamp+self.segment_frames/50.>=1:
            self.stop('segment_exceeds_action_horizon');return False
        try:self.chunk=validate_actions(result['actions']).copy()
        except (ValueError,TypeError):self.stop('invalid_prediction');return False
        self.observed_at=stamp;self.active_request_id=self.pending['request_id'];self.next_frame=0
        self.phase='ready';self.pending=None
        return True if self.defer_execution else self.begin(now)
    def begin(self,now):
        if not self.check(now) or self.phase!='ready':return False
        if not fresh(self.observed_at,now,.95) or now-self.observed_at+self.segment_frames/50.>=1:
            self.stop('segment_exceeds_action_horizon');return False
        self.execution_at=now;self.phase='execute';return True
    def action(self,now,sensors_ok=True):
        if not self.check(now,sensors_ok) or self.phase!='execute':return None
        if not 0<=now-self.observed_at<1:self.stop('action_expired');return None
        elapsed=(now-self.execution_at)*50
        due=int(math.floor(elapsed+1e-8))
        if self.next_frame==self.segment_frames:
            if due>=self.segment_frames:
                self.not_before=now;self.phase='observe';self.chunk=None
            return None
        if due<self.next_frame:return None
        if due>self.next_frame:self.stop('missed_execution_deadline');return None
        i=self.next_frame;self.next_frame+=1
        return dict(window_index=i,action=self.chunk[i].copy(),observed_at=self.observed_at,
                    execution_at=self.execution_at,generation=self.generation,
                    request_id=self.active_request_id)
