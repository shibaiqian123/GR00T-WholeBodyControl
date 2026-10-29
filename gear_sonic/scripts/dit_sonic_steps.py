"""Manual capture -> saved-observation inference -> OFFLINE SONIC decode.
No action publishers, robot startup, mode switch, or automatic next stage.
"""
import argparse
import base64
import hashlib
import io
import json
import subprocess
import time
import uuid
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]

def save_json(path,data):
    path.write_text(json.dumps(data,indent=2,allow_nan=False))

def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(1<<20),b''):h.update(b)
    return h.hexdigest()

def array(value,n,name):
    a=np.asarray(value,dtype=float)
    if a.shape!=(n,) or not np.isfinite(a).all():raise ValueError(f'{name}: expected {n} finite values')
    return a

def validate_snapshot(s):
    schemas={'sonic_v1_1_prospective_control_v1':'WAIT_FOR_CONTROL',
             'sonic_v1_1_planner_observation_v1':'CONTROL_PLANNER'}
    if s.get('kind')!='readonly_decode_snapshot' or schemas.get(s.get('schema_name'))!=s.get('state'):
        raise ValueError('Missing supported complete bottom snapshot; no history will be fabricated')
    array(s['decoder_input'],994,'decoder_input')
    for k in ('actual_q','actual_dq','scale','default_angles'):array(s[k],29,k)
    m=array(s['mapping'],29,'mapping')
    if sorted(m.tolist())!=list(range(29)):raise ValueError('Invalid mapping')
    if not np.isfinite(float(s['sample_time'])):raise ValueError('Invalid snapshot timestamp')
    if s['state']=='WAIT_FOR_CONTROL' and 'padding_rows' not in s:raise ValueError('Native history padding provenance missing')
    return s

def tail_snapshot(path):
    with path.open('rb') as f:
        f.seek(0,2);start=max(0,f.tell()-4*1024*1024);f.seek(start)
        if start:f.readline()
        lines=f.read().splitlines()
    for line in reversed(lines):
        try:s=json.loads(line)
        except (ValueError,UnicodeDecodeError):continue
        if s.get('kind')=='readonly_decode_snapshot':return validate_snapshot(s)
    raise ValueError('No complete readonly_decode_snapshot in log tail')

def paired(snapshot,diag,body,now):
    validate_snapshot(snapshot)
    stamp=float(snapshot['sample_time'])
    if not 0<=now-stamp<=.25:raise ValueError('Bottom snapshot older than 250ms or future timestamp')
    times=[stamp,float(body['sample_time']),now-diag['camera_age_seconds']]
    times += [now-diag[s+'_feedback_age_seconds'] for s in ('left','right')]
    if max(times)-min(times)>.15:raise ValueError('Snapshot/image/body/gripper skew exceeds 150ms')

def capture(a):
    import msgpack,zmq
    from gear_sonic.camera.composed_camera import ComposedCameraClientSensor
    from gear_sonic.data.robot_model.instantiation.g1 import instantiate_g1_robot_model
    from gear_sonic.scripts.run_dex1_trigger_control import read_limits
    from gear_sonic.utils.inference.dit4dit_control import sonic_state
    from gear_sonic.utils.inference.readonly_body import ReadOnlyBodySubscriber
    from gear_sonic.utils.inference.observation_diagnostics import inspect_observation
    if a.bottom_log is not None and not a.bottom_log.is_file():raise FileNotFoundError('Missing bottom snapshot log. No bottom will be started automatically.')
    if a.output.exists():raise FileExistsError(a.output)
    limits=read_limits(a.limits);robot=instantiate_g1_robot_model(waist_location='lower_and_upper_body')
    ctx=zmq.Context();dex=ctx.socket(zmq.SUB);dex.setsockopt(zmq.LINGER,0);dex.setsockopt(zmq.SUBSCRIBE,b'dex1_telemetry ');dex.connect('tcp://127.0.0.1:5559')
    body=ReadOnlyBodySubscriber(5564);camera=ComposedCameraClientSensor('127.0.0.1',5555)
    latest=None;failure='waiting for sensors';deadline=time.monotonic()+a.wait_seconds
    try:
        while time.monotonic()<deadline:
            for _ in range(100):
                try:raw=dex.recv(zmq.NOBLOCK)
                except zmq.Again:break
                try:latest=msgpack.unpackb(raw[len(b'dex1_telemetry '):],raw=False)
                except (ValueError,TypeError):latest=None
            bm=body.get_msg();cm=camera.read();now=time.monotonic()
            d=inspect_observation(cm,bm,latest,limits,now,time.time(),body.cache.error)
            try:
                if not d['ok']:raise ValueError('; '.join(d['reasons']))
                s=tail_snapshot(a.bottom_log) if a.bottom_log else None
                if s is not None:paired(s,d,bm,time.monotonic())
                # Align VLA body state to exactly the body sample used by SONIC.
                q=array(s['actual_q'] if s else bm['body_q'],29,'actual_q')
                whole=robot.get_configuration_from_actuated_joints(body_actuated_joint_values=q)
                state=sonic_state(whole,[latest['sides'][side]['q'] for side in ('left','right')])
                observed=now-d['camera_age_seconds']
                a.output.mkdir(parents=True,exist_ok=False)
                np.savez(a.output/'observation.npz',image=np.asarray(cm['images']['ego_view']).copy(),state=state,
                    observed_at=observed,body_sample_time=s['sample_time'] if s else bm['sample_time'],body_source_tick=s['source_tick'] if s else bm['source_tick'],
                    dex_feedback_time=[latest['sides'][side]['feedback_time'] for side in ('left','right')])
                if s is not None:save_json(a.output/'sonic_snapshot.json',s)
                save_json(a.output/'manifest.json',dict(schema=1,task_prompt=a.task_prompt,captured_at=time.monotonic(),
                    command_publishers_created=False,bottom_log=str(a.bottom_log.resolve()) if a.bottom_log else None,
                    sonic_state=s['state'] if s else None,complete_decoder_input_saved=s is not None,decoder_context=('prospective_native_history' if s['state']=='WAIT_FOR_CONTROL' else 'captured_planner_input') if s else 'not_captured',
                    files={name:sha(a.output/name) for name in (('observation.npz','sonic_snapshot.json') if s else ('observation.npz',))},diagnostic=d,
                    note='VLA body uses bottom actual_q; independent body subscriber validates live feedback. No next stage executed.'))
                print('CAPTURE_SAVED',a.output);return
            except (ValueError,KeyError,TypeError) as e:failure=str(e)
            time.sleep(.02)
        raise RuntimeError('Capture failed; nothing fabricated: '+failure)
    finally:
        camera.close();body.close();dex.close();ctx.term()

def load_bundle(path):
    m=json.loads((path/'manifest.json').read_text())
    for name in m['files']:
        if sha(path/name)!=m['files'][name]:raise ValueError('Capture file hash mismatch: '+name)
    s=validate_snapshot(json.loads((path/'sonic_snapshot.json').read_text())) if 'sonic_snapshot.json' in m['files'] else None
    return m,s

def infer(a):
    import zmq
    from PIL import Image
    m,_=load_bundle(a.capture)
    with np.load(a.capture/'observation.npz',allow_pickle=False) as n:
        state=array(n['state'],31,'state');image=n['image'];observed=float(n['observed_at'])
    if image.dtype!=np.uint8 or image.ndim!=3 or image.shape[2]!=3:raise ValueError('Expected RGB uint8 image')
    b=io.BytesIO();Image.fromarray(image).convert('RGB').resize((224,224),Image.Resampling.BILINEAR).save(b,format='PNG')
    req=dict(op='infer',generation=0,request_id=uuid.uuid4().hex,observed_at=observed,state=state.tolist(),lang=m['task_prompt'],image_png=base64.b64encode(b.getvalue()).decode())
    a.output.mkdir(parents=True,exist_ok=False)
    save_json(a.output/'request.json',req)
    save_json(a.output/'source.json',dict(capture=str(a.capture.resolve()),manifest_hash=sha(a.capture/'manifest.json'),saved_observation=True,not_for_live_execution=True))
    with zmq.Context() as ctx:
        sock=ctx.socket(zmq.REQ);sock.setsockopt(zmq.LINGER,0);sock.connect('tcp://127.0.0.1:5560')
        try:
            start=time.monotonic();sock.send_json(req)
            if not sock.poll(10000):raise TimeoutError('No inference reply. No automatic retry or robot action.')
            r=sock.recv_json();r['client_roundtrip_seconds']=time.monotonic()-start
        finally:sock.close()
    save_json(a.output/'response.json',r)
    if not r.get('ok'):raise ValueError(r.get('error','inference failed'))
    if r.get('request_id')!=req['request_id'] or r.get('generation')!=0 or r.get('observed_at')!=observed:raise ValueError('Reply identity mismatch')
    for name in ('actions','normalized'):
        v=np.asarray(r[name]);
        if v.shape!=(50,66) or not np.isfinite(v).all():raise ValueError('Invalid '+name)
        np.save(a.output/(name+'.npy'),v)
    np.save(a.output/'token_step0.npy',np.asarray(r['actions'])[0,:64]);np.save(a.output/'gripper_step0.npy',np.asarray(r['actions'])[0,64:])
    print('PREDICTION_SAVED',a.output,'shape=50x66; first step saved; NO SEND')

def decode(a):
    _,s=load_bundle(a.capture)
    if a.sonic_snapshot is not None:s=validate_snapshot(json.loads(a.sonic_snapshot.read_text()))
    if s is None:raise ValueError('Complete SONIC input missing. Use snapshot subcommand then --sonic-snapshot; no zero filling.')
    source=json.loads((a.prediction/'source.json').read_text())
    if source['manifest_hash']!=sha(a.capture/'manifest.json'):raise ValueError('Prediction belongs to a different capture')
    r=json.loads((a.prediction/'response.json').read_text());req=json.loads((a.prediction/'request.json').read_text())
    if not r.get('ok') or r.get('request_id')!=req.get('request_id'):raise ValueError('Invalid prediction provenance')
    token=array(np.load(a.prediction/'token_step0.npy',allow_pickle=False),64,'token')
    actions=np.asarray(r['actions']);
    if actions.shape!=(50,66) or not np.isfinite(actions).all() or not np.array_equal(token,actions[0,:64]):raise ValueError('Not the saved first frame')
    x=array(s['decoder_input'],994,'decoder input').copy();x[:64]=token
    x=x.astype('<f4')
    if not np.isfinite(x).all():raise ValueError('float32 overflow')
    a.output.mkdir(parents=True,exist_ok=False);x.tofile(a.output/'input.f32')
    save_json(a.output/'resources.json',{str(p.resolve()):sha(p) for p in (a.decoder,a.model,a.capture/'manifest.json',a.prediction/'response.json')})
    raw_path=a.output/'raw_action.f32'
    with (a.output/'decoder.log').open('w') as log:
        subprocess.run([str(a.decoder.resolve()),str(a.model.resolve()),str((a.output/'input.f32').resolve()),str(raw_path.resolve())],stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
    raw=array(np.fromfile(raw_path,dtype='<f4'),29,'raw action');meta=json.loads(Path(str(raw_path)+'.mapping.json').read_text())
    for k in ('mapping','scale','default_angles'):
        if not np.allclose(array(meta[k],29,k),array(s[k],29,k),atol=1e-12,rtol=0):raise ValueError('Decoder mapping mismatch: '+k)
    target=(raw[np.asarray(s['mapping'],dtype=int)]*np.asarray(s['scale'])+np.asarray(s['default_angles'])).astype(np.float32).astype(float)
    grippers=array(np.load(a.prediction/'gripper_step0.npy',allow_pickle=False),2,'grippers')
    if not np.array_equal(grippers,actions[0,64:]):raise ValueError('Gripper first-frame provenance mismatch')
    action31=np.concatenate((target,grippers))
    np.save(a.output/'body_target29.npy',target);np.save(a.output/'action31.npy',action31)
    delta=target-np.asarray(s['actual_q']);idx=int(np.argmax(abs(delta)))
    save_json(a.output/'result.json',dict(kind='saved_token_and_saved_body_offline_decode',motor_written=False,executed=False,
        action31=action31.tolist(),action31_layout='hardware_body_29_radians_then_left_Dex1_right_Dex1_radians',grippers_sent=False,
        context_state=s['state'],context_schema=s['schema_name'],padding_rows=s.get('padding_rows'),
        original_sample_time=s['sample_time'],raw_action=raw.tolist(),actual_q=s['actual_q'],target=target.tolist(),delta=delta.tolist(),
        max_delta_joint_index=idx,max_abs_delta=float(abs(delta[idx])),position_and_transition_limits_verified=False))
    print('DECODE_SAVED',a.output,'max_abs_delta=',float(abs(delta[idx])),'joint index=',idx,'NO MOTOR SEND')

def snapshot(a):
    s=tail_snapshot(a.bottom_log)
    if not 0<=time.monotonic()-float(s['sample_time'])<=.25:raise ValueError('No fresh bottom snapshot (250ms gate)')
    with a.output.open('x') as f:json.dump(s,f,indent=2,allow_nan=False)
    print('SONIC_SNAPSHOT_SAVED',a.output,'state=',s['state'])

def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='stage',required=True)
    c=sub.add_parser('capture');c.add_argument('--bottom-log',type=Path);c.add_argument('--task-prompt',required=True);c.add_argument('--output',type=Path,required=True);c.add_argument('--limits',type=Path,default=ROOT/'gear_sonic/configs/dex1_limits.g1_20261013.json');c.add_argument('--wait-seconds',type=float,default=10)
    i=sub.add_parser('infer');i.add_argument('--capture',type=Path,required=True);i.add_argument('--output',type=Path,required=True)
    d=sub.add_parser('decode');d.add_argument('--capture',type=Path,required=True);d.add_argument('--prediction',type=Path,required=True);d.add_argument('--output',type=Path,required=True);d.add_argument('--decoder',type=Path,default=ROOT/'gear_sonic_deploy/target/release/replay_sonic_decoder');d.add_argument('--model',type=Path,default=ROOT/'gear_sonic_deploy/policy/sonic_v1_1/model_decoder.onnx')
    d.add_argument('--sonic-snapshot',type=Path,help='Explicit saved complete bottom input; may be sampled after inference')
    b=sub.add_parser('snapshot');b.add_argument('--bottom-log',type=Path,required=True);b.add_argument('--output',type=Path,required=True)
    a=p.parse_args();{'capture':capture,'infer':infer,'decode':decode,'snapshot':snapshot}[a.stage](a)
if __name__=='__main__':main()
