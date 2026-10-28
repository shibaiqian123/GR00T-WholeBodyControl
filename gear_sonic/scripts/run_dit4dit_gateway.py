"""229-only live observation and exclusive PICO/DiT4DiT arbitration.
Default is read-only: no robot PUB sockets are created. --execute still starts readonly.
Commands (Enter): teleop, model, stop, reset, quit. PICO takeover supports PLANNER
modes; POSE packets are deliberately rejected to avoid mixing protocol v3 and v4.
"""
import argparse
import base64
import io
import json
import queue
import select
import sys
import threading
import time
from pathlib import Path
import numpy as np
from gear_sonic.utils.inference.dit4dit_control import Arbiter, fresh, sonic_state, command_fields


def worker(endpoint, requests, replies, done):
    import zmq
    ctx = zmq.Context()
    while not done.is_set():
        try:
            req = requests.get(timeout=.1)
        except queue.Empty:
            continue
        sock = ctx.socket(zmq.REQ)
        sock.setsockopt(zmq.LINGER,0)
        sock.connect(endpoint)
        try:
            sock.send_json(req)
            if not sock.poll(2000):
                raise TimeoutError('Inference service timeout')
            reply = sock.recv_json()
            if reply.get('request_id') != req['request_id'] or reply.get('generation') != req['generation']:
                raise ValueError('Mismatched inference response')
            replies.put(reply)
        except Exception as exc:
            replies.put(dict(ok=False,error=str(exc),generation=req['generation']))
        finally:
            sock.close()
    ctx.term()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task-prompt',required=True)
    p.add_argument('--limits',type=Path,required=True)
    p.add_argument('--camera-port',type=int,default=5555)
    p.add_argument('--state-port',type=int,default=5557)
    p.add_argument('--dex-telemetry-port',type=int,default=5559)
    p.add_argument('--pico-port',type=int,default=5562)
    p.add_argument('--policy-port',type=int,default=5560)
    p.add_argument('--action-port',type=int,default=5556)
    p.add_argument('--target-port',type=int,default=5563)
    p.add_argument('--log-dir',type=Path,required=True)
    p.add_argument('--duration',type=float,default=0,help='Seconds, 0 until quit')
    p.add_argument('--execute',action='store_true')
    a=p.parse_args()
    import zmq, msgpack
    from PIL import Image
    from gear_sonic.camera.composed_camera import ComposedCameraClientSensor
    from gear_sonic.data.robot_model.instantiation.g1 import instantiate_g1_robot_model
    from gear_sonic.utils.data_collection.zmq_state_subscriber import ZMQStateSubscriber
    from gear_sonic.scripts.run_dex1_trigger_control import PacketReader, read_limits
    from gear_sonic.utils.teleop.zmq.zmq_planner_sender import build_command_message, pack_pose_message
    ports=[a.camera_port,a.state_port,a.dex_telemetry_port,a.pico_port,a.policy_port,a.action_port,a.target_port]
    if len(set(ports))!=len(ports) or not all(1024<=x<=65535 for x in ports):
        p.error('All ports must be distinct and unprivileged')
    limits=read_limits(a.limits)
    a.log_dir.mkdir(parents=True,exist_ok=False)
    log=(a.log_dir/'events.jsonl').open('w',buffering=1)
    def record(kind, **data):
        row=dict(time=time.monotonic(),kind=kind,**data)
        log.write(json.dumps(row,allow_nan=False)+'\n')
        if kind not in ('prediction','frame'):
            print(json.dumps(row),flush=True)
    robot=instantiate_g1_robot_model(waist_location='lower_and_upper_body')
    ctx=zmq.Context(); sockets=[]
    def sub(port, topic):
        s=ctx.socket(zmq.SUB);s.setsockopt(zmq.LINGER,0);s.setsockopt(zmq.RCVHWM,100)
        s.setsockopt(zmq.SUBSCRIBE,topic);s.connect(f'tcp://127.0.0.1:{port}');sockets.append(s)
        return s
    pico=sub(a.pico_port,b'')
    hb_socket=sub(a.state_port,b'dex1_control_status ')
    hb=PacketReader(hb_socket,b'dex1_control_status ')
    dex=sub(a.dex_telemetry_port,b'dex1_telemetry ')
    pub=targets=None
    if a.execute:
        def publisher(port):
            s=ctx.socket(zmq.PUB);s.setsockopt(zmq.LINGER,0);s.setsockopt(zmq.SNDHWM,5)
            s.bind(f'tcp://127.0.0.1:{port}');sockets.append(s);return s
        pub=publisher(a.action_port);targets=publisher(a.target_port)
    camera=ComposedCameraClientSensor('127.0.0.1',a.camera_port)
    body=ZMQStateSubscriber('127.0.0.1',a.state_port)
    arb=Arbiter();requests=queue.Queue(maxsize=1);replies=queue.Queue();done=threading.Event()
    thread=threading.Thread(target=worker,args=(f'tcp://127.0.0.1:{a.policy_port}',requests,replies,done),daemon=True)
    thread.start()
    body_msg=None;body_at=-1.;dex_msg=None;planner=None;planner_at=-1.
    pico_planner=False;trigger=None;busy=False;last_request=-1.;request_id=0;frame_id=0
    last_mode='readonly';stop_sent=False;started=time.monotonic();last_health=-1.
    record('startup',execute=a.execute,mode=arb.mode,controls='teleop/model/stop/reset/quit + Enter')
    try:
        while not a.duration or time.monotonic()-started<a.duration:
            tick=time.monotonic();arb.status(hb.poll(),tick)
            bm=body.get_msg()
            if bm is not None:
                body_msg=bm;body_at=tick
            for _ in range(100):
                try:
                    raw=dex.recv(zmq.NOBLOCK)
                except zmq.Again:
                    break
                try:
                    candidate=msgpack.unpackb(raw[len(b'dex1_telemetry '):],raw=False)
                    if fresh(candidate['time'],tick) and (not dex_msg or candidate['time']>dex_msg['time']):
                        dex_msg=candidate
                except Exception:
                    dex_msg=None
            # Never silently renew a sample's age with its receive timestamp.
            cm=camera.read();now=time.monotonic();wall=time.time()
            sensor_ok=False;state=None;observed_at=None;img=None;image_age=None
            try:
                image_stamp=float(cm['timestamps']['ego_view'])
                image_age=wall-image_stamp
                feedback=[dex_msg['sides'][s] for s in ('left','right')]
                sensor_ok=(0<=image_age<=.25 and fresh(body_at,now)
                           and fresh(dex_msg['time'],now)
                           and all(x['feedback_valid'] is True and fresh(x['feedback_time'],now) for x in feedback))
                if sensor_ok:
                    whole=robot.get_configuration_from_actuated_joints(body_actuated_joint_values=np.asarray(body_msg['body_q']))
                    state=sonic_state(whole,[x['q'] for x in feedback])
                    observed_at=now-image_age
                    stamps=[observed_at,body_at]+[x['feedback_time'] for x in feedback]
                    sensor_ok=max(stamps)-min(stamps)<=.15
                    img=np.asarray(cm['images']['ego_view'])
                    sensor_ok=sensor_ok and img.ndim==3 and img.shape[2]==3 and img.dtype==np.uint8
            except (KeyError,TypeError,ValueError,AttributeError):
                sensor_ok=False
            dex_ready=bool(dex_msg and fresh(dex_msg['time'],now) and dex_msg.get('phase')==4)
            for _ in range(100):
                try:
                    raw=pico.recv(zmq.NOBLOCK)
                except zmq.Again:
                    break
                try:
                    command=command_fields(raw)
                    if command is not None:
                        if command['stop'] or (arb.mode=='model' and command['start']):
                            arb.stop('pico_stop_or_start_during_model')
                        pico_planner=command['planner']
                        if arb.mode=='teleop' and a.execute:
                            if not command['planner']:
                                arb.stop('PICO_POSE_not_supported_in_token_session')
                            elif not command['stop']:
                                pub.send(raw)
                    elif raw.startswith(b'dex1_trigger '):
                        candidate=msgpack.unpackb(raw[len(b'dex1_trigger '):],raw=False)
                        if candidate.get('stop') is True:
                            arb.stop('pico_stop')
                        if not trigger or candidate['time']>trigger['time']:
                            trigger=candidate
                    elif raw.startswith(b'planner'):
                        planner=raw;planner_at=now
                except Exception as exc:
                    arb.stop('invalid_pico_packet');record('input_error',error=str(exc))
            pico_ok=bool(pico_planner and planner and fresh(planner_at,now,.1) and trigger
                         and trigger.get('valid') is True and trigger.get('stop') is False
                         and fresh(trigger.get('time'),now) and fresh(trigger.get('sample_time'),now)
                         and all(isinstance(trigger.get(s),(int,float)) and np.isfinite(trigger[s]) and 0<=trigger[s]<=1 for s in ('left','right')))
            try:
                result=replies.get_nowait();busy=False
                record('prediction',**result)
                if result.get('generation')==arb.generation:
                    if result.get('ok'):
                        arb.accept(result,now)
                    elif arb.mode=='model':
                        arb.stop('inference_failure')
            except queue.Empty:
                pass
            if sys.stdin.isatty() and select.select([sys.stdin],[],[],0)[0]:
                key=sys.stdin.readline().strip().lower()
                try:
                    if key=='quit':
                        break
                    if key=='stop':
                        arb.stop('manual_stop')
                    elif key=='reset':
                        arb.reset(now);stop_sent=False
                    elif key in ('model','teleop'):
                        if not a.execute:
                            raise ValueError('Read-only process: restart with --execute to enable outputs')
                        arb.select(key,now,sensor_ok,pico_ok,dex_ready)
                        if key=='model':
                            pub.send(build_command_message(False,False,False))
                        else:
                            pub.send(planner)
                            pub.send(build_command_message(True,False,True))
                except ValueError as exc:
                    record('command_rejected',error=str(exc))
            if sensor_ok and arb.mode!='stop' and not busy and now-last_request>=.4:
                buffer=io.BytesIO();Image.fromarray(img).save(buffer,format='PNG')
                req=dict(op='infer',generation=arb.generation,request_id=request_id,
                         observed_at=observed_at,state=state.tolist(),lang=a.task_prompt,
                         image_png=base64.b64encode(buffer.getvalue()).decode('ascii'))
                np.savez_compressed(a.log_dir/f'input_{request_id:06d}.npz',image=img,state=state,observed_at=observed_at)
                requests.put_nowait(req);busy=True;last_request=now;request_id+=1
            if arb.mode=='model' and not dex_ready:
                arb.stop('dex1_not_ready')
            action=arb.frame(now,sensor_ok)
            target=None
            if arb.mode=='teleop':
                if not pico_ok or not arb.bottom_ok(now):
                    arb.stop('pico_or_bottom_stale')
                elif pub:
                    pub.send(planner)
                    target={s:limits[s].command_open+trigger[s]*(limits[s].closed-limits[s].command_open) for s in ('left','right')}
            elif action is not None and pub:
                pub.send(pack_pose_message({'token_state':action[:64].reshape(1,64),
                                             'frame_index':np.array([frame_id],dtype=np.int64)},topic='pose',version=4))
                target=dict(left=float(action[64]),right=float(action[65]))
                record('frame',index=frame_id,mode=arb.mode,token=action[:64].tolist(),targets=target)
            if target is not None and targets:
                stamp=time.monotonic()
                packet=dict(seq=frame_id,time=stamp,sample_time=stamp,valid=True,stop=False,mode=arb.mode,**target)
                targets.send(b'dex1_target '+msgpack.packb(packet,use_bin_type=True))
            if arb.mode=='stop' and pub:
                pub.send(build_command_message(False,True,True))
                targets.send(b'dex1_target '+msgpack.packb(dict(time=time.monotonic(),sample_time=0.,valid=False,stop=True),use_bin_type=True))
                stop_sent=True
            if last_mode!=arb.mode:
                record('mode',mode=arb.mode,reason=arb.reason,generation=arb.generation);last_mode=arb.mode
            if now-last_health>=1:
                record('health',mode=arb.mode,sensors_ok=bool(sensor_ok),pico_ok=pico_ok,dex_ready=dex_ready,busy=busy,
                       camera_age_seconds=image_age,body_age_seconds=now-body_at if body_msg else None,
                       dex_feedback_present=bool(dex_msg and all(dex_msg['sides'][s].get('feedback_valid') for s in ('left','right'))))
                last_health=now
            frame_id+=1
            time.sleep(max(0.,.02-(time.monotonic()-tick)))
    except KeyboardInterrupt:
        pass
    finally:
        if pub:
            pub.send(build_command_message(False,True,True))
            targets.send(b'dex1_target '+msgpack.packb(dict(time=time.monotonic(),sample_time=0.,valid=False,stop=True),use_bin_type=True))
        done.set();thread.join(timeout=2.5)
        camera.close();body.close()
        for s in sockets:s.close()
        ctx.term();record('exit');log.close()

if __name__=='__main__':
    main()
