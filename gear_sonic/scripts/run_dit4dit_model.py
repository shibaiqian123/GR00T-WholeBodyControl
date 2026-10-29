"""Pure model gateway; manual arm/model/stop, no PICO subscriptions."""
import argparse,base64,contextlib,io,json,queue,select,sys,threading,time,uuid
from pathlib import Path
import numpy as np
from gear_sonic.utils.inference.model_only_control import ModelControl
from gear_sonic.utils.inference.sync_model_control import SyncModelControl
from gear_sonic.utils.inference.dit4dit_control import fresh,sonic_state
from gear_sonic.utils.inference.remote_policy_client import worker
from gear_sonic.utils.inference.observation_diagnostics import inspect_observation


def poll_bottom_status(control, reader, clock=time.monotonic):
    """Evaluate freshness after receiving; never compare a packet to a pre-read clock."""
    began=clock()
    packet=reader.poll()
    checked=clock()
    control.status(packet,checked)
    stamp=(packet or {}).get('time')
    return dict(checked_at=checked, poll_seconds=checked-began,
                packet_time=stamp, age_seconds=checked-stamp if isinstance(stamp,(int,float)) else None,
                seq=(packet or {}).get('seq'), state=(packet or {}).get('state'),
                stop=(packet or {}).get('stop'), reader_error=getattr(reader,'last_error',None))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--limits',type=Path,required=True);p.add_argument('--task-prompt',required=True)
    p.add_argument('--log-dir',type=Path,required=True);p.add_argument('--execute',action='store_true')
    p.add_argument('--duration',type=float,default=0)
    p.add_argument('--inference-mode',choices=('async','sync'),default='async')
    p.add_argument('--segment-frames',type=int,default=10,help='Sync: frames per segment at 50 Hz; last token continues during inference')
    for name,port in [('camera',5555),('state',5557),('dex',5559),('policy',5560),('action',5556),('target',5563),('body',5564)]:
        p.add_argument('--'+name+'-port',type=int,default=port)
    a=p.parse_args();ports=[getattr(a,k+'_port') for k in ('camera','state','dex','policy','action','target','body')]
    if len(set(ports))!=len(ports) or any(not 1024<=x<=65535 for x in ports):p.error('Ports must be distinct, unprivileged')
    if not 1<=a.segment_frames<=50:p.error('--segment-frames must be 1..50')
    if a.inference_mode=='sync' and a.segment_frames==50:p.error('50 sync frames leave no inference time inside the unchanged 1s action horizon')
    a.log_dir.mkdir(parents=True,exist_ok=False)
    ui=sys.stdout
    with (a.log_dir/'library.log').open('w',buffering=1) as noisy,contextlib.redirect_stdout(noisy):
        run(a,ui)


def run(a,ui):
    import zmq,msgpack
    from PIL import Image
    from gear_sonic.camera.composed_camera import ComposedCameraClientSensor
    from gear_sonic.data.robot_model.instantiation.g1 import instantiate_g1_robot_model
    from gear_sonic.scripts.run_dex1_trigger_control import PacketReader,read_limits
    from gear_sonic.utils.inference.readonly_body import ReadOnlyBodySubscriber
    from gear_sonic.utils.teleop.zmq.zmq_planner_sender import build_command_message,pack_pose_message
    log=(a.log_dir/'events.jsonl').open('w',buffering=1)
    def record(kind,**data):log.write(json.dumps(dict(time=time.monotonic(),kind=kind,**data),allow_nan=False)+'\n')
    def show(s):print(s,file=ui,flush=True)
    saved_terminal=None
    if sys.stdin.isatty():
        import termios
        saved_terminal=termios.tcgetattr(sys.stdin.fileno());new=list(saved_terminal);new[0]&=~termios.IXON
        termios.tcsetattr(sys.stdin.fileno(),termios.TCSANOW,new)
    limits=read_limits(a.limits);robot=instantiate_g1_robot_model(waist_location='lower_and_upper_body')
    ctx=zmq.Context();sockets=[]
    def socket(kind,port,topic=None):
        s=ctx.socket(kind);s.setsockopt(zmq.LINGER,0);sockets.append(s)
        if kind==zmq.SUB:
            s.setsockopt(zmq.RCVHWM,100);s.setsockopt(zmq.SUBSCRIBE,topic);s.connect(f'tcp://127.0.0.1:{port}')
        else:s.setsockopt(zmq.SNDHWM,5);s.bind(f'tcp://127.0.0.1:{port}')
        return s
    hb=PacketReader(socket(zmq.SUB,a.state_port,b'dex1_control_status '),b'dex1_control_status ')
    dex=socket(zmq.SUB,a.dex_port,b'dex1_telemetry ')
    pub=socket(zmq.PUB,a.action_port) if a.execute else None
    targets=socket(zmq.PUB,a.target_port) if a.execute else None
    body=ReadOnlyBodySubscriber(a.body_port);camera=ComposedCameraClientSensor('127.0.0.1',a.camera_port)
    control=SyncModelControl(a.segment_frames) if a.inference_mode=='sync' else ModelControl();requests=queue.Queue(1);replies=queue.Queue();done=threading.Event()
    thread=threading.Thread(target=worker,args=(f'tcp://127.0.0.1:{a.policy_port}',requests,replies,done),daemon=True);thread.start()
    dex_msg=None;busy=False;last_request=-1.;rid=0;frame=0;last_health=-1.;last_mode=None;arm_id=None;held=None;arm_at=0.;start_sent=False
    begin=time.monotonic();last_summary=None;last_sensor_reasons=None;previous_tick=None
    show('MODEL ONLY. Commands + Enter: arm (hold current grippers), model (MOVE), stop, recover, reset, quit. No VR. Logs: '+str(a.log_dir))
    record('startup',execute=a.execute,body_source='independent_dds',bottom_watchdog_port=a.state_port,inference_mode=a.inference_mode,segment_frames=a.segment_frames)
    show('Inference: '+a.inference_mode+(' — serial segments; SONIC continues last token while waiting, NOT stationary hold' if a.inference_mode=='sync' else ''))
    try:
        while not a.duration or time.monotonic()-begin<a.duration:
            tick=time.monotonic();heartbeat_diagnostic=poll_bottom_status(control,hb);bm=body.get_msg()
            for _ in range(100):
                try:raw=dex.recv(zmq.NOBLOCK)
                except zmq.Again:break
                try:
                    d=msgpack.unpackb(raw[len(b'dex1_telemetry '):],raw=False)
                    if fresh(d['time'],time.monotonic()) and (dex_msg is None or d['time']>dex_msg['time']):dex_msg=d
                except (ValueError,KeyError,TypeError):dex_msg=None
            cm=camera.read();now=time.monotonic();wall=time.time();state=None;observed=None;image=None;feedback=[]
            diagnostic=inspect_observation(cm,bm,dex_msg,limits,now,wall,body.cache.error)
            diagnostic['bottom_heartbeat']=heartbeat_diagnostic
            diagnostic['loop_gap_seconds']=None if previous_tick is None else tick-previous_tick
            diagnostic['observation_read_seconds']=now-tick
            previous_tick=tick
            sensor_ok=diagnostic['ok']
            if sensor_ok:
                try:
                    observed=now-diagnostic['camera_age_seconds']
                    feedback=[dex_msg['sides'][s] for s in ('left','right')]
                    whole=robot.get_configuration_from_actuated_joints(body_actuated_joint_values=np.asarray(bm['body_q']))
                    state=sonic_state(whole,[x['q'] for x in feedback]);image=np.asarray(cm['images']['ego_view'])
                except (ValueError,TypeError,KeyError,AttributeError) as exc:
                    sensor_ok=False;diagnostic['ok']=False
                    diagnostic['reasons'].append('state_mapping:'+type(exc).__name__+':'+str(exc))
            reasons=tuple(diagnostic['reasons'])
            if reasons!=last_sensor_reasons:
                record('observation_validity',mode=control.mode,diagnostic=diagnostic)
                last_sensor_reasons=reasons
            ready=bool(dex_msg and fresh(dex_msg['time'],now) and dex_msg.get('phase')==4)
            if control.mode=='armed' and (not sensor_ok or (now-arm_at>2 and not ready)):
                record('stop_diagnostic',mode=control.mode,diagnostic=diagnostic,dex_ready=ready,arm_age_seconds=now-arm_at)
                show('STOP DETAIL: '+(', '.join(diagnostic['reasons']) or 'dex_ready_timeout'))
                control.stop('armed_feedback_invalid')
            try:
                result=replies.get_nowait();busy=False
                try:
                    record('prediction',**result)
                except (ValueError,TypeError):
                    record('prediction_error',error='Non-finite or non-serializable prediction')
                    if isinstance(result,dict) and result.get('generation')==control.generation:
                        control.stop('invalid_prediction')
                    result={}
                if result.get('generation')==control.generation:
                    if result.get('ok'):
                        try:control.accept(result,now)
                        except (ValueError,TypeError,KeyError):control.stop('invalid_prediction')
                    elif control.mode in ('starting','model') or (a.inference_mode=='sync' and control.mode=='armed'):control.stop('model_error')
            except queue.Empty:pass
            if sys.stdin.isatty() and select.select([sys.stdin],[],[],0)[0]:
                key=sys.stdin.readline().strip().lower()
                if key=='quit':break
                try:
                    if not a.execute and key in ('arm','model'):raise ValueError('Read-only process; --execute required')
                    control.command(key,now,sensor_ok,ready)
                    if key=='arm':
                        arm_id=uuid.uuid4().hex;held={s:float(x['q']) for s,x in zip(('left','right'),feedback)};arm_at=now
                    if key=='model':
                        start_sent=False;pub.send(build_command_message(False,False,False))
                    record('command',command=key,mode=control.mode);show('OK '+key+' -> '+control.mode+(' (waiting for a fresh prediction)' if a.inference_mode=='sync' and key=='model' else ''))
                except ValueError as exc:record('command_rejected',command=key,error=str(exc));show('REJECTED: '+str(exc))
            sync_request=None
            if a.inference_mode=='sync' and sensor_ok and not busy:
                sync_request=control.request_metadata(rid,observed,now)
            request_allowed=(sync_request is not None) if a.inference_mode=='sync' else (control.mode!='stop' and now-last_request>=.35)
            if sensor_ok and not busy and request_allowed:
                raw=dict(image=image.copy(),state=state.copy(),observed_at=observed,
                         body_sample_time=bm['sample_time'],body_source_tick=bm['source_tick'],
                         dex_feedback_time=[x['feedback_time'] for x in feedback])
                req=dict(op='infer',generation=control.generation,request_id=rid,observed_at=observed,
                         state=state.tolist(),lang=a.task_prompt,_input=raw,
                         _input_path=str(a.log_dir/f'input_{rid:06d}.npz'))
                if sync_request is not None:req.update({k:sync_request[k] for k in ('generation','request_id','observed_at')})
                record('inference_request',request_id=req['request_id'],generation=req['generation'],observed_at=req['observed_at'],inference_mode=a.inference_mode)
                requests.put_nowait(req);busy=True;last_request=now;rid+=1
            action=control.action(now,sensor_ok,ready)
            target=None;target_mode=None
            if control.mode=='armed':target=held;target_mode='arm'
            if action is not None:
                if not all(limits[s].contains(float(action[64+i])) for i,s in enumerate(('left','right'))):
                    control.stop('gripper_prediction_out_of_limits');action=None
                else:
                    pub.send(pack_pose_message({'token_state':action[:64].reshape(1,64),'frame_index':np.array([frame],dtype=np.int64)},topic='pose',version=4))
                    record('frame',index=frame,mode=control.mode,token=action[:64].tolist(),sync=getattr(control,'action_info',None))
                    if control.mode=='starting':
                        target=held;target_mode='hold'
                        if not start_sent and now-control.started>=.1 and control.bottom.get('token_ready') is True:
                            pub.send(build_command_message(True,False,False));start_sent=True;record('bottom_start')
                        if start_sent and control.bottom['state']=='CONTROL':control.mode='model';control.reason='bottom_control_ack'
                    if control.mode=='model':target={s:float(action[64+i]) for i,s in enumerate(('left','right'))};target_mode='model'
            if target is not None and control.mode!='stop' and targets:
                targets.send(b'dex1_target '+msgpack.packb(dict(time=now,sample_time=now,seq=frame,valid=True,stop=False,mode=target_mode,arm_id=arm_id,**target),use_bin_type=True))
            if control.mode=='stop' and pub:
                if not control.stop_ack:pub.send(build_command_message(False,True,False))
                targets.send(b'dex1_target '+msgpack.packb(dict(time=now,sample_time=0.,valid=False,stop=True),use_bin_type=True))
            if last_mode!=control.mode:
                record('mode',mode=control.mode,reason=control.reason,generation=control.generation,diagnostic=diagnostic);show('STATE '+control.mode+': '+control.reason);last_mode=control.mode
            if now-last_health>=1:
                summary=(bool(sensor_ok),ready,control.bottom.get('state') if control.bottom else None,bool(control.chunk and fresh(control.chunk[0],now,.5)))
                record('health',mode=control.mode,sensors_ok=bool(sensor_ok),dex_ready=ready,bottom=control.bottom,body_error=body.cache.error,busy=busy,diagnostic=diagnostic,sync_phase=control.scheduler.phase if a.inference_mode=='sync' else None)
                if summary!=last_summary:show('sensors=%s dex_ready=%s bottom=%s prediction_fresh=%s'%summary);last_summary=summary
                last_health=now
            frame+=1;time.sleep(max(0.,.02-(time.monotonic()-tick)))
    except KeyboardInterrupt:pass
    finally:
        if pub:
            if control.mode in ('armed','starting','model') or (control.mode=='stop' and not control.stop_ack):pub.send(build_command_message(False,True,False))
            targets.send(b'dex1_target '+msgpack.packb(dict(time=time.monotonic(),sample_time=0.,valid=False,stop=True),use_bin_type=True))
        done.set();thread.join(timeout=2.5);camera.close();body.close()
        for s in sockets:s.close()
        ctx.term();record('exit');log.close()
        if saved_terminal is not None:termios.tcsetattr(sys.stdin.fileno(),termios.TCSANOW,saved_terminal)
        show('Gateway exited. No automatic restart.')

if __name__=='__main__':main()
