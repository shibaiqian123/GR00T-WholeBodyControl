"""Independent proportional-trigger Dex1-1 position controller. Dry-run unless --execute is explicitly supplied.

All three processes must run on the same Linux host (shared monotonic clock).
This process stays alive in FAULT_HOLD after the body controller exits.
"""
import argparse
import json
import math
import select
import sys
import threading
import time
from pathlib import Path

from gear_sonic.utils.teleop.dex1_trigger import Limits, RampController, SIDES

class DdsGrippers:
    def __init__(self, interface, domain=0, execute=False, kp=5., kd=.05, sdk=None):
        if sdk is None:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber, ChannelPublisher
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_, MotorCmd_
            sdk = (ChannelFactoryInitialize, ChannelSubscriber, ChannelPublisher, MotorCmds_, MotorStates_, MotorCmd_)
        init, subscriber, publisher, self.command_type, state_type, self.motor_type = sdk
        self.measured = {}; self.lock = threading.Lock()
        self.publishers = {}; self.subscribers = []; self.kp=kp; self.kd=kd
        init(domain, interface)
        for side in SIDES:
            sub = subscriber(f'rt/dex1/{side}/state', state_type)
            sub.Init(lambda msg, s=side: self.on_state(s,msg), 1)
            self.subscribers.append(sub)
            if execute:
                pub=publisher(f'rt/dex1/{side}/cmd',self.command_type);pub.Init()
                self.publishers[side]=pub
    def on_state(self, side, msg):
        try:
            q=float(msg.states[0].q) if len(msg.states)==1 else math.nan
            dq=float(msg.states[0].dq) if len(msg.states)==1 else math.nan
        except (AttributeError,TypeError,ValueError):
            q=dq=math.nan
        with self.lock:
            self.measured[side]=(q,time.monotonic(),dq)
    def snapshot(self):
        with self.lock:return dict(self.measured)
    def write(self, targets):
        for side,q in targets.items():
            if side in self.publishers:
                motor=self.motor_type(mode=1,q=float(q),dq=0.,tau=0.,kp=self.kp,kd=self.kd,reserve=[0,0,0])
                if self.publishers[side].Write(self.command_type(cmds=[motor]),timeout=.01) is False:
                    raise RuntimeError(f'{side} DDS write failed; cannot guarantee holding')
    def close(self):
        for channel in self.subscribers+list(self.publishers.values()):channel.Close()

class PacketReader:
    """Validate source time as well as receipt time; repeated/stale packets cannot refresh watchdog."""
    def __init__(self, socket, prefix, trigger=False):
        self.socket=socket;self.prefix=prefix;self.trigger=trigger;self.latest=None
        self.last_time=-math.inf
        self.last_error=""
    def poll(self, now=None):
        import msgpack,zmq
        fault = None
        for _ in range(100):
            try:raw=self.socket.recv(flags=zmq.NOBLOCK)
            except zmq.Again:break
            try:
                packet_now = time.monotonic() if now is None else now
                if not raw.startswith(self.prefix):raise ValueError('topic')
                p=msgpack.unpackb(raw[len(self.prefix):],raw=False)
                if not isinstance(p,dict):raise ValueError('map')
                stamp=p['time']
                if not isinstance(stamp,(int,float)) or not math.isfinite(stamp):raise ValueError('time')
                if not isinstance(p.get('seq'),int) or p['seq']<0:raise ValueError('seq')
                if stamp>packet_now+.001 or packet_now-stamp>.25:raise ValueError(f'source clock/stale packet: age={packet_now-stamp:.3f}s')
                if stamp<=self.last_time:continue
                if self.trigger:
                    if type(p.get('valid')) is not bool or type(p.get('stop')) is not bool:raise ValueError('flags')
                    t=p.get('sample_time')
                    if not isinstance(t,(int,float)) or not math.isfinite(t):raise ValueError('sample time')
                    if p['valid'] and not all(isinstance(p.get(s),(int,float)) and math.isfinite(p[s]) and 0<=p[s]<=1 for s in SIDES):raise ValueError('trigger')
                elif p.get('state') not in ('INIT','WAIT_FOR_CONTROL','CONTROL','STOP') or type(p.get('stop')) is not bool:
                    raise ValueError('controller status')
                self.last_time=stamp;self.latest=p;self.last_error=""
                if p.get("stop") or (self.trigger and not p["valid"]):
                    fault = p  # Preserve the stop edge while draining queued packets.
            except (ValueError,TypeError,KeyError,OverflowError,msgpack.ExtraData,msgpack.FormatError,msgpack.StackError) as exc:
                self.last_error=str(exc)
                invalid_time=time.monotonic() if now is None else now
                fault={'time':invalid_time,'sample_time':invalid_time,'valid':False,'stop':True,'state':'STOP'}
        return fault if fault is not None else self.latest

def read_limits(path):
    data=json.loads(path.read_text())
    if data.get('units') != 'motor_output_radians':
        raise ValueError('Limits must explicitly use motor_output_radians')
    if data.get('calibrated') is not True:
        raise ValueError('Limits must be confirmed calibrated; do not use demonstration angles')
    return {s:Limits(float(data[s]['closed_rad']),float(data[s]['open_rad']),float(data[s].get('open_margin_rad',0.0))) for s in SIDES}

def demo():
    c=RampController({s:Limits(0.,1.) for s in SIDES})
    measured=dict.fromkeys(SIDES,0.5)
    for i in range(401):
        t=i*.02
        trigger={'time':t,'sample_time':t,'valid':True,'stop':False,'left':float(150<=i<250),'right':float(180<=i<230)}
        status={'time':t,'state':'WAIT_FOR_CONTROL','stop':i>=350}
        q=c.update(t,{s:(v,t) for s,v in measured.items()},trigger,status)
        measured.update(q)
        if i%50==0:print(f'{t:4.1f}s {c.phase:16s} q={q}')
    print('DEMO ONLY: normalized virtual endpoints; no network or motor commands')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--interface',help='Explicit robot NIC; no auto-detection')
    p.add_argument('--domain',type=int,default=0)
    p.add_argument('--limits',type=Path,help='JSON with confirmed calibrated endpoints, radians')
    p.add_argument('--startup-open-time','--travel-time',dest='travel_time',type=float,default=2.,help='Startup opening time only; normal trigger tracking is direct, without slew limiting')
    p.add_argument('--obstruction-hold',action='store_true',help='Opt-in per-hand blocked-closing hold; release trigger to captured aperture to resume')
    p.add_argument('--obstruction-timeout',type=float,default=1.,help='Continuous blocked-closing duration in seconds (default: 1)')
    p.add_argument('--feedback-timeout',type=float,default=.25)
    p.add_argument('--trigger-port',type=int,default=5556)
    p.add_argument('--status-port',type=int,default=5557)
    p.add_argument('--kp',type=float,default=5.)
    p.add_argument('--kd',type=float,default=.05)
    p.add_argument('--observe',action='store_true',help='Read-only Dex1 feedback for ten seconds; no endpoint file required')
    p.add_argument('--execute',action='store_true',help='Enable real DDS motor commands; otherwise receive-only dry-run')
    p.add_argument('--demo',action='store_true',help='Offline demonstration with virtual feedback; no DDS/ZMQ')
    a=p.parse_args()
    if a.observe and (a.execute or a.demo):p.error("--observe cannot be combined with --execute or --demo")
    if a.demo:
        if a.execute:p.error('--demo cannot be combined with --execute')
        demo();return
    if not a.interface:p.error('--interface is required')
    if not a.observe and not a.limits:p.error('--limits is required')
    if not 0<=a.domain<=232:p.error('DDS domain must be in [0,232]')
    if not all(math.isfinite(x) and x>=0 for x in (a.kp,a.kd)):p.error('Gains must be finite and nonnegative')
    if not 0<a.feedback_timeout<=.25:p.error('Feedback timeout must be in (0,.25] seconds')
    if a.observe:
        dds=DdsGrippers(a.interface,a.domain,execute=False)
        try:
            for _ in range(10):
                print(json.dumps(dds.snapshot()),flush=True);time.sleep(1)
        finally:dds.close()
        return
    try:c=RampController(read_limits(a.limits),a.travel_time,a.feedback_timeout,
                         obstruction_hold=a.obstruction_hold,obstruction_timeout=a.obstruction_timeout)
    except (ValueError,TypeError,KeyError,OSError) as exc:p.error(str(exc))
    import zmq
    context=zmq.Context();sockets=[]
    for port,topic in [(a.trigger_port,b'dex1_trigger '),(a.status_port,b'dex1_control_status ')]:
        sock=context.socket(zmq.SUB);sock.setsockopt(zmq.LINGER,0);sock.setsockopt(zmq.RCVHWM,5)
        sock.setsockopt(zmq.SUBSCRIBE,topic);sock.connect(f'tcp://127.0.0.1:{port}');sockets.append(sock)
    triggers=PacketReader(sockets[0],b'dex1_trigger ',True)
    status=PacketReader(sockets[1],b'dex1_control_status ')
    dds=None
    print('EXECUTE: motor commands enabled' if a.execute else 'DRY-RUN: no DDS command publishers are created',flush=True)
    print('R + Enter: resume a latched hold, only with fresh data and both triggers released. Q + Enter: exit (holding ceases).',flush=True)
    try:
        dds=DdsGrippers(a.interface,a.domain,a.execute,a.kp,a.kd)
        last_report=-math.inf;deadline=time.monotonic();previous=None
        while True:
            now=time.monotonic();resume=False
            if sys.stdin.isatty() and select.select([sys.stdin],[],[],0)[0]:
                key=sys.stdin.readline().strip().lower()
                if key=='q':break
                resume=key=='r'
            tr=triggers.poll();hb=status.poll();measured=dds.snapshot()
            now=time.monotonic()
            targets=c.update(now,measured,tr,hb,resume)
            dds.write(targets)
            report_state=(c.phase, tuple(v['state'] for v in c.obstruction_status.values()) if c.obstruction_status else ())
            if now-last_report>=1 or previous!=report_state:
                print(json.dumps({'mode':'execute' if a.execute else 'dry-run','state':c.phase,'reason':c.reason,
                    'input_errors':[triggers.last_error,status.last_error],
                    'obstruction_status':c.obstruction_status,
                    'feedback_status':c.feedback_status,'first_fault':c.first_fault,
                    'trigger':{s:tr.get(s) for s in SIDES} if tr else None,
                    'measured':{s:v[0] for s,v in measured.items()},'target':targets,
                    'requested_target':c.requested if c.phase=='READY' else None,
                    'endpoint_braking':c.endpoint_braking if c.phase=='READY' else None,
                    'direction':c.motion if c.phase=='READY' else None}),flush=True)
                previous=report_state;last_report=now
            deadline+=.02
            if now-deadline>.04:deadline=now
            time.sleep(max(0.,deadline-time.monotonic()))
    except KeyboardInterrupt:
        print('Exiting: position holding cannot continue after this process closes.')
    finally:
        if dds:dds.close()
        for sock in sockets:sock.close()
        context.term()
if __name__=='__main__':main()
