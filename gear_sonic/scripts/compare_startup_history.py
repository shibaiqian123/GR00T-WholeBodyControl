"""Offline fixed-token history ablation. Never imports DDS or deployment entrypoints."""
import argparse,json,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

SEGMENTS={'gyro':(64,94,3),'q':(94,384,29),'dq':(384,674,29),'action':(674,964,29),'gravity':(964,994,3)}

def array(row,key,shape):
    a=np.asarray(row.get(key,[]),dtype=np.float64)
    if a.shape!=shape or not np.isfinite(a).all():raise ValueError('Missing/invalid '+key)
    return a

def measured_history(first,sim):
    obs=array(first,'decoder_input',(994,));mapping=array(first,'mapping',(29,)).astype(int)
    if sorted(mapping)!=list(range(29)):raise ValueError('Invalid permutation')
    inverse=np.argsort(mapping);defaults=array(first,'default_angles',(29,))
    actual=array(first,'actual_q',(29,))
    candidates=[s for s in sim if s.get('kind')=='sim_state' and 0<=first['time']-s['time']<=.1 and np.allclose(np.asarray(s['body_q'],dtype=np.float32),actual,atol=1e-7,rtol=0)]
    if not candidates:raise ValueError('No matching pre-decoder simulator sample; cannot reconstruct measured history')
    anchor=max(candidates,key=lambda s:s['time'])
    prior=sorted([s for s in sim if s.get('kind')=='sim_state' and s['time']<=anchor['time']],key=lambda s:s['time'])
    rows=[];sources=[]
    for i in range(9):
        target=anchor['time']-(9-i)*.02
        eligible=[s for s in prior if s['time']<=target+1e-9]
        if not eligible:raise ValueError('Missing measured pre-start history')
        s=eligible[-1]
        if target-s['time']>.01:raise ValueError('Measured history gap exceeds 10ms')
        if s['sim_time']>=anchor['sim_time']:raise ValueError('Simulator time reset in history window')
        q=array(s,'body_q',(29,)).astype(np.float32).astype(float)
        dq=array(s,'body_dq',(29,)).astype(np.float32).astype(float)
        pos=np.asarray(s['qpos']);vel=np.asarray(s['qvel'])
        if len(pos)<7 or len(vel)<6 or not np.isfinite(pos).all() or not np.isfinite(vel).all():raise ValueError('Missing base state')
        quat=pos[3:7].astype(np.float32).astype(float)
        if abs(np.linalg.norm(quat)-1)>.001:raise ValueError('Invalid base quaternion')
        gravity=Rotation.from_quat(quat[[1,2,3,0]]).inv().apply([0,0,-1])
        # Same q/dq/gyro fields published by UnitreeSdk2Bridge; no invented sensor zeros.
        rows.append(dict(gyro=vel[3:6].astype(np.float32),q=(q-defaults)[inverse],dq=dq[inverse],gravity=gravity))
        sources.append(dict(time=s['time'],sim_time=s['sim_time'],requested_time=target,sample_age=target-s['time']))
    replacement={key:np.asarray([r[key] for r in rows]) for key in ('gyro','q','dq','gravity')}
    return replacement,dict(anchor_candidates=len(candidates),chosen_anchor=anchor['time'],selection='Latest matching sample; timing ambiguous if multiple candidates',samples=sources,note='Resampled simulator measurements, not an exact captured DDS history; original first sample and all last_action entries preserved')

def variants(first,replacement):
    original=array(first,'decoder_input',(994,)).astype(np.float32)
    cases={'captured_original':original.copy()}
    repeated=original.copy()
    for key,(start,end,width) in SEGMENTS.items():
        if key=='action':continue
        view=repeated[start:end].reshape(10,width);view[:9]=view[-1]
    cases['synthetic_repeat_current_state']=repeated
    for label,keys in [('measured_q_only',['q']),('measured_motion_only',['gyro','dq']),('measured_gravity_only',['gravity']),('measured_state_history',['gyro','q','dq','gravity'])]:
        v=original.copy()
        for key in keys:
            start,end,width=SEGMENTS[key];v[start:end].reshape(10,width)[:9]=replacement[key]
        cases[label]=v
    for v in cases.values():
        if not np.isfinite(v).all():raise ValueError('Nonfinite variant')
        assert np.array_equal(v[:64],original[:64])
        assert np.array_equal(v[674:964],original[674:964])
        for start,end,width in SEGMENTS.values():assert np.array_equal(v[end-width:end],original[end-width:end])
    return cases

def read(path):return [json.loads(s) for s in path.read_text().splitlines()]

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--capture',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    root=Path(__file__).resolve().parents[2]/'gear_sonic_deploy';engine=root/'target/release/replay_sonic_decoder';decoder=root/'policy/sonic_v1_1/model_decoder.onnx'
    report=[]
    for case in json.loads((a.capture/'results.json').read_text()):
        source=Path(case['output']);rows=read(source/'bottom.jsonl');first=next(r for r in rows if r.get('kind')=='decoder_command');sim=read(source/'sim.jsonl')
        replacements,provenance=measured_history(first,sim);cases=variants(first,replacements)
        directory=a.output/case['case'];directory.mkdir()
        inp=directory/'inputs.f32';output=directory/'actions.f32';np.stack(list(cases.values())).tofile(inp)
        with (directory/'engine.log').open('w') as log:subprocess.run([str(engine),str(decoder),str(inp),str(output)],stdout=log,stderr=subprocess.STDOUT,check=True)
        actions=np.fromfile(output,dtype=np.float32).reshape(len(cases),29)
        if not np.isfinite(actions).all():raise ValueError('Nonfinite decoder output')
        if not np.allclose(actions[0],first['raw_action'],rtol=1e-3,atol=1e-4):raise ValueError('Baseline parity failed')
        mapping=np.asarray(first['mapping'],dtype=int);scale=np.asarray(first['scale']);defaults=np.asarray(first['default_angles']);q=np.asarray(first['actual_q'])
        compiled=json.loads(Path(str(output)+'.mapping.json').read_text())
        for key in ('mapping','scale','default_angles'):
            if not np.allclose(compiled[key],first[key],atol=1e-12,rtol=0):raise ValueError('Compiled mapping mismatch')
        results=[]
        for (label,inp_row),action in zip(cases.items(),actions):
            target=defaults+action[mapping]*scale;delta=target-q
            results.append(dict(variant=label,raw_action=action.tolist(),target=target.tolist(),delta=delta.tolist(),max_delta_rad=float(np.max(abs(delta))),largest_joint=int(np.argmax(abs(delta))),rms_delta_rad=float(np.sqrt(np.mean(delta**2))),left_shoulder_roll_delta=float(delta[16]),right_elbow_delta=float(delta[25]),changed_input_dimensions=int(np.count_nonzero(inp_row!=cases['captured_original']))))
        report.append(dict(case=case['case'],source_capture_passed=case['passed'],source_footer=rows[-1],token_unchanged=True,last_action_unchanged=True,current_state_unchanged=True,history_provenance=provenance,baseline_max_error=float(np.max(abs(actions[0]-np.asarray(first['raw_action'])))),actual_q=q.tolist(),results=results))
    (a.output/'comparison.json').write_text(json.dumps(report,indent=2))
    print(json.dumps([dict(case=r['case'],baseline_error=r['baseline_max_error'],variants=[{k:v for k,v in x.items() if k not in ('raw_action','target','delta')} for x in r['results']]) for r in report],indent=2))
if __name__=='__main__':main()
