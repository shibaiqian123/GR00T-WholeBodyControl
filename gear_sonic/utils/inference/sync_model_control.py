"""Serial VLA cycles; SONIC continues decoding the last token while waiting.

This does NOT promise a stationary pose. All repeated targets retain their
original observation deadline; no waiting phase can extend the 1s horizon.
"""
from gear_sonic.utils.inference.model_only_control import ModelControl
from gear_sonic.utils.inference.synchronous_policy import SynchronousPolicy

class SyncModelControl(ModelControl):
    def __init__(self,segment_frames):
        super().__init__()
        self.segment_frames=segment_frames
        self.scheduler=SynchronousPolicy(segment_frames,defer_execution=True)
        self.last_output=None;self.action_info=None;self.model_requested=False
    def stop(self,reason):
        super().stop(reason)
        self.scheduler.stop(reason);self.last_output=None;self.action_info=None;self.model_requested=False
    def command(self,key,now,sensors=False,dex_ready=False):
        if key=='model':
            if self.mode!='armed' or self.model_requested or not sensors or not dex_ready or not self.bottom_ok(now) or self.bottom['state']!='WAIT_FOR_CONTROL':
                raise ValueError('Sync model needs armed, fresh sensors/READY grippers, WAIT_FOR_CONTROL and no pending start')
            self.model_requested=True;self.chunk=None;self.reason='sync_waiting_prediction'
            return
        super().command(key,now,sensors,dex_ready)
        if key=='reset':
            self.scheduler=SynchronousPolicy(self.segment_frames,defer_execution=True)
            self.scheduler.generation=self.generation
            self.last_output=None;self.action_info=None;self.model_requested=False
    def request_metadata(self,rid,observed,now):
        if self.mode not in ('armed','model') or self.scheduler.phase!='observe':return None
        if self.mode=='armed' and not self.model_requested:return None
        req=self.scheduler.request(observed,now)
        if req is None:return None
        self.scheduler.pending['request_id']=rid
        return dict(self.scheduler.pending)
    def accept(self,result,now):
        if self.mode not in ('armed','model') or result.get('generation')!=self.generation:return False
        ok=self.scheduler.accept(result,now)
        if self.scheduler.phase=='stop':self.stop(self.scheduler.reason)
        if ok:
            self.chunk=(self.scheduler.observed_at,self.scheduler.chunk)
            if self.mode=='armed':
                self.mode='starting';self.started=now;self.reason='sync_prefill_first_frame'
        return ok
    def action(self,now,sensors,dex_ready):
        self.action_info=None
        if self.mode not in ('starting','model'):return None
        if not sensors or not dex_ready or not self.bottom_ok(now):
            self.stop('input_or_gripper_not_ready');return None
        if self.mode=='starting':
            if now-self.started>2:self.stop('start_timeout');return None
            if self.bottom['state']=='CONTROL':
                if not self.scheduler.begin(now):self.stop(self.scheduler.reason);return None
                self.mode='model';self.reason='bottom_control_ack'
            else:
                if not self.chunk or not 0<=now-self.chunk[0]<1:
                    self.stop('action_expired');return None
                self.action_info=dict(phase='prefill',window_index=0,request_id=self.scheduler.active_request_id,observed_at=self.chunk[0])
                return self.chunk[1][0].copy()
        if self.scheduler.phase=='ready':
            if not self.scheduler.begin(now):self.stop(self.scheduler.reason);return None
        packet=self.scheduler.action(now,sensors)
        if self.scheduler.phase=='stop':self.stop(self.scheduler.reason);return None
        if packet is not None:self.last_output=packet
        if self.last_output is None:self.stop('missing_sync_action');return None
        old=self.last_output
        if not 0<=now-old['observed_at']<1:
            self.stop('action_expired');return None
        self.action_info=dict(phase='segment' if packet is not None else 'continue_last_token',
            scheduler_phase=self.scheduler.phase,window_index=old['window_index'],
            request_id=old['request_id'],observed_at=old['observed_at'])
        return old['action'].copy()
