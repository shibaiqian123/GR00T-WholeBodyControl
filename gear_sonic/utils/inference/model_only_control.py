"""Explicit model-only lifecycle. No I/O or automatic re-arming."""
import math
import numpy as np
from gear_sonic.utils.inference.dit4dit_control import fresh, validate_actions

class ModelControl:
    def __init__(self):
        self.mode='readonly';self.reason='startup';self.generation=0
        self.bottom=None;self.chunk=None;self.stop_ack=False;self.started=0.
    def bottom_ok(self,now):
        p=self.bottom or {}
        return fresh(p.get('time'),now) and p.get('stop') is False and p.get('state') in ('WAIT_FOR_CONTROL','CONTROL')
    def stop(self,reason):
        if self.mode!='stop':
            self.generation+=1;self.chunk=None;self.stop_ack=False;self.reason=reason
        self.mode='stop'
    def status(self,p,now):
        self.bottom=p
        if p and fresh(p.get('time'),now) and p.get('stop') is True:
            self.stop('bottom_stop');self.stop_ack=True
        elif self.mode in ('armed','starting','model') and not self.bottom_ok(now):
            self.stop('bottom_stale')
    def command(self,key,now,sensors=False,dex_ready=False):
        if key=='stop':self.stop('manual_stop');return
        if key=='recover':
            if self.mode!='stop':raise ValueError('recover only after stop')
            # Explicitly cease body STOP retries only. Motion remains latched off.
            self.stop_ack=True;return
        if key=='reset':
            if self.mode!='stop' or not self.stop_ack or not self.bottom_ok(now) or self.bottom['state']!='WAIT_FOR_CONTROL':
                raise ValueError('Need stopped latch, acknowledged stop/recover, and restarted WAIT_FOR_CONTROL')
            self.generation+=1;self.chunk=None;self.mode='readonly';self.reason='manual_reset';return
        if self.mode=='stop':raise ValueError('Stop latched: recover, restart bottom, then reset')
        if key=='arm':
            if self.mode!='readonly' or not sensors or not self.bottom_ok(now) or self.bottom['state']!='WAIT_FOR_CONTROL':
                raise ValueError('arm needs fresh sensors and bottom WAIT_FOR_CONTROL')
            self.mode='armed';self.reason='manual_hold_current_grippers';return
        if key=='model':
            failures=[]
            if self.mode!='armed':failures.append('mode='+self.mode+' (need armed)')
            if not sensors:failures.append('sensors invalid')
            if not dex_ready:failures.append('grippers not READY')
            if not self.bottom_ok(now):failures.append('bottom heartbeat invalid/stale')
            if (self.bottom or {}).get('state')!='WAIT_FOR_CONTROL':failures.append('bottom not WAIT_FOR_CONTROL')
            if self.chunk is None:failures.append('prediction missing')
            elif not fresh(self.chunk[0],now,.5):failures.append('prediction age=%.3fs (need 0..0.500s)'%(now-self.chunk[0]))
            if failures:raise ValueError('; '.join(failures))
            self.mode='starting';self.started=now;self.reason='prefill_real_token';return
        raise ValueError('Commands: arm, model, stop, recover, reset, quit')
    def accept(self,result,now):
        if self.mode=='stop' or result.get('generation')!=self.generation or not fresh(result.get('observed_at'),now,.95):return False
        actions=validate_actions(result['actions']);self.chunk=(result['observed_at'],actions);return True
    def action(self,now,sensors,dex_ready):
        if self.mode not in ('starting','model'):return None
        if not sensors or not dex_ready or not self.bottom_ok(now):self.stop('input_or_gripper_not_ready');return None
        if self.mode=='starting' and now-self.started>2:self.stop('start_timeout');return None
        if not self.chunk or not 0 <= now-self.chunk[0] < 1:self.stop('action_expired');return None
        return self.chunk[1][int((now-self.chunk[0])*50)].copy()

class ModelGripInput:
    """Opt-in manual hold-current arm; existing RampController handles ongoing protection."""
    def __init__(self, *, allow_default_reference_control=False):
        self.seen=set();self.active=None;self.hold=None
        self.allow_default_reference_control=allow_default_reference_control
    def update(self,controller,packet,measured,heartbeat,now):
        p=packet or {};key=p.get('arm_id');mode=p.get('mode')
        valid=(p.get('valid') is True and p.get('stop') is False and fresh(p.get('time'),now) and fresh(p.get('sample_time'),now)
               and isinstance(key,str) and bool(key) and mode in ('arm','hold','model'))
        if valid and mode=='arm' and key not in self.seen:
            self.seen.add(key)
            good=(heartbeat and fresh(heartbeat.get('time'),now) and heartbeat.get('stop') is False
                  and (heartbeat.get('state')=='WAIT_FOR_CONTROL' if not self.allow_default_reference_control else
                       heartbeat.get('state')=='CONTROL' and type(heartbeat.get('external_stream')) is int and heartbeat['external_stream']==0))
            for side in ('left','right'):
                m=measured.get(side)
                good=good and m is not None and len(m)>=3 and all(math.isfinite(x) for x in m) and controller._fresh(m[1],now) and controller.limits[side].contains(m[0])
            if good and controller.phase in ('WAIT_INPUTS','FAULT_HOLD'):
                self.active=key;self.hold={s:float(measured[s][0]) for s in ('left','right')}
                controller.target=dict(self.hold);controller.phase='READY';controller.reason='';controller.first_fault=None
                controller.motion=dict.fromkeys(self.hold,'holding');controller.last_time=now
                controller.hold_captured.clear()
                for guard in controller.obstruction.values():guard.reset()
        if not valid or key!=self.active:
            p=dict(time=now,sample_time=0.,valid=False,stop=True)
        elif mode in ('arm','hold'):
            p=dict(p)
            for s,q in self.hold.items():
                lim=controller.limits[s];p[s]=max(0.,min(1.,(q-lim.command_open)/(lim.closed-lim.command_open)))
        # Never let a missing arm fall through to the legacy auto-opening path.
        if controller.phase=='WAIT_INPUTS':return {}
        return controller.update(now,measured,p,heartbeat,resume=False)
