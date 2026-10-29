"""Offline startup provenance audit. No sockets, DDS or controller imports."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np

def match_token(token,predictions,at,atol=1e-7):
    matches=[]
    for p in predictions:
        if p['time']>at or not p.get('ok'):continue
        a=np.asarray(p.get('actions',[]))
        if a.shape!=(50,66):continue
        for i in np.flatnonzero(np.all(np.isclose(a[:,:64],token,rtol=0,atol=atol),axis=1)):
            matches.append(dict(request_id=p['request_id'],generation=p.get('generation'),window_index=int(i),observed_at=p['observed_at'],observation_age=at-p['observed_at'],max_error=float(np.max(np.abs(a[i,:64]-token)))))
    return dict(status='unique' if len(matches)==1 else 'ambiguous' if matches else 'unmatched',candidates=matches)

def audit(root):
    events=[json.loads(l) for l in (root/'gateway/events.jsonl').open()]
    starts=[e for e in events if e['kind']=='bottom_start'];out=[]
    for start in starts:
        t=start['time'];pred=[e for e in events if e['kind']=='prediction' and t-4<=e['time']<=t+2]
        frames=[e for e in events if e['kind']=='frame' and t-2<=e['time']<=t+2]
        result=dict(run=str(root),start_time=t,events=[{k:v for k,v in e.items() if k!='diagnostic'} for e in events if e['kind'] in ('mode','command','bottom_start') and t-2<=e['time']<=t+2],frames=[])
        arms=[e for e in events if e['kind']=='command' and e.get('command')=='arm' and e['time']<=t]
        result['preceding_arm']=arms[-1] if arms else None
        for f in frames:
            result['frames'].append(dict(time=f['time'],send_counter=f['index'],phase=f['mode'],match=match_token(f['token'],pred,f['time'])))
        inputs=[]
        for p in pred:
            path=root/'gateway'/('input_%06d.npz'%p['request_id'])
            if not path.exists():continue
            with np.load(path,allow_pickle=False) as a:
                state=a['state'];stamp=float(a['observed_at'])
                if not t-2<=stamp<=t+2:continue
                inputs.append(dict(request_id=p['request_id'],observed_at=stamp,body_sample_time=float(a['body_sample_time']),state31=state.tolist(),valid=state.shape==(31,) and bool(np.isfinite(state).all()),fields=list(a.files)))
        result['inputs']=inputs
        result['first_prefill']=next((f for f in result['frames'] if f['phase']=='starting'),None)
        result['first_gateway_model_frame']=next((f for f in result['frames'] if f['phase']=='model'),None)
        result['evidence_limits']=['Gateway model phase is heartbeat acknowledgment, not proof of first decoder execution.','NPZ state is sampled state, not exact activation-time state.','No historical complete decoder input/raw action/final DDS target in this event schema.','Per-joint target deltas cannot be reconstructed without missing decoder records.']
        result['first_executed_target_delta']=None
        out.append(result)
    return out

def enrich(report, stats_path, normalization_source):
    import importlib.util
    from gear_sonic.data.robot_model.instantiation.g1 import instantiate_g1_robot_model
    from gear_sonic.utils.inference.dit4dit_control import sonic_state
    spec=importlib.util.spec_from_file_location('audit_normalization',normalization_source)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    stats=json.loads(stats_path.read_text())['new_embodiment']
    robot=instantiate_g1_robot_model(waist_location='lower_and_upper_body')
    perm=sonic_state(robot.get_configuration_from_actuated_joints(body_actuated_joint_values=np.arange(29,dtype=float)),[101.,102.])[:29].astype(int)
    if sorted(perm)!=list(range(29)):raise ValueError('State mapping is not a joint permutation')
    report['training_state_body_permutation']=perm.tolist()
    report['normalization_source_sha256']=sha(normalization_source)
    for run in report['starts']:
        errors=[]
        for line in (Path(run['run'])/'gateway/events.jsonl').open():
            e=json.loads(line)
            if e['kind']=='prediction' and e.get('ok') and run['start_time']-4<=e['time']<=run['start_time']+2:
                restored=module.unnormalize_sonic_actions(np.asarray(e['normalized'],dtype=np.float32),stats['action']).astype(np.float32)
                errors.append(float(np.max(np.abs(restored-np.asarray(e['actions'],dtype=np.float32)))))
        run['recorded_unnormalization_max_error']=max(errors) if errors else None
        prior=[x for x in run['inputs'] if x['body_sample_time']<=run['start_time']]
        if prior:
            x=max(prior,key=lambda x:x['body_sample_time']);q=np.empty(29);q[perm]=np.asarray(x['state31'])[:29]
            run['nearest_prior_actual_sample']=dict(sample_time=x['body_sample_time'],age_at_start=run['start_time']-x['body_sample_time'],hardware_q=q.tolist(),gripper_motor_radians=x['state31'][29:],note='Sampled state, not exact activation-time feedback')


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(8*1024*1024),b''):h.update(b)
    return h.hexdigest()

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--runs',nargs='+',type=Path,required=True);p.add_argument('--assets',nargs='*',type=Path,default=[]);p.add_argument('--output',type=Path,required=True);p.add_argument('--statistics',type=Path);p.add_argument('--normalization-source',type=Path);a=p.parse_args()
    report=dict(schema=1,starts=sorted([x for root in a.runs for x in audit(root)],key=lambda x:x['start_time']),assets=[dict(path=str(x),sha256=sha(x),bytes=x.stat().st_size) for x in a.assets])
    if bool(a.statistics)!=bool(a.normalization_source):p.error('statistics and normalization-source must be provided together')
    if a.statistics:enrich(report,a.statistics,a.normalization_source)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2,allow_nan=False));print(a.output)
if __name__=='__main__':main()
