"""Offline decoder-input replay. Never imports DDS or robot deployment entrypoints."""
import argparse,json,subprocess,tempfile
from pathlib import Path
import numpy as np

def validated(row):
    required={'decoder_input':994,'raw_action':29,'joint_target':29,'actual_q':29,'mapping':29,'scale':29,'default_angles':29}
    for key,n in required.items():
        a=np.asarray(row.get(key,[]))
        if a.shape!=(n,) or not np.isfinite(a).all():raise ValueError('Missing/invalid '+key+'; historical replay unavailable, no zero filling')
    mapping=np.asarray(row['mapping'])
    if not np.array_equal(np.sort(mapping),np.arange(29)):raise ValueError('Invalid joint permutation')
    return row

def map_action(action,row):
    return np.asarray(row['default_angles'])+np.asarray(action)[np.asarray(row['mapping'],dtype=int)]*np.asarray(row['scale'])

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trace',required=True,type=Path);p.add_argument('--decoder',required=True,type=Path);p.add_argument('--output',required=True,type=Path);p.add_argument('--engine-runner',type=Path,default=Path(__file__).resolve().parents[2]/'gear_sonic_deploy/target/release/replay_sonic_decoder');a=p.parse_args()
    rows=[json.loads(x) for x in a.trace.open()]
    commands=[validated(r) for r in rows if r.get('kind')=='decoder_command']
    if not commands:raise ValueError('No complete decoder_command records; cannot replay historical token-only logs')
    ends=[r for r in rows if r.get('kind')=='audit_end']
    if len(ends)!=1 or ends[0].get('dropped')!=0 or rows[-1].get('kind')!='audit_end':raise ValueError('Incomplete/dropped/appended diagnostic trace')
    with tempfile.TemporaryDirectory(prefix='sonic_offline_') as directory:
        inp=Path(directory)/'observations.f32';out=Path(directory)/'actions.f32'
        np.asarray([r['decoder_input'] for r in commands],dtype=np.float32).tofile(inp)
        subprocess.run([str(a.engine_runner),str(a.decoder),str(inp),str(out)],check=True)
        actions=np.fromfile(out,dtype=np.float32).reshape(len(commands),29)
        compiled=json.loads(Path(str(out)+'.mapping.json').read_text())
        for row in commands:
            for key in ('mapping','scale','default_angles'):
                if not np.allclose(row[key],compiled[key],rtol=0,atol=1e-12):raise ValueError('Runtime mapping differs from captured '+key)
    results=[]
    for r,action in zip(commands,actions):
        if action.shape!=(29,) or not np.isfinite(action).all():raise ValueError('Invalid decoder output')
        expected=np.asarray(r['raw_action']);target=map_action(action,r);actual=np.asarray(r['actual_q'])
        mapped_record=map_action(expected,r)
        if not np.allclose(mapped_record,r['joint_target'],rtol=1e-6,atol=1e-6):raise ValueError('Recorded action mapping inconsistent')
        # Declared before replay, never tune against observed discrepancies.
        passed=bool(np.allclose(action,expected,rtol=1e-3,atol=1e-4))
        delta=target-actual
        writes=[w for w in rows if w.get('kind')=='dds_write_attempt' and r['time']<=w['time']<=r['time']+.1 and np.asarray(w.get('q',[])).shape==(29,) and np.allclose(w['q'],r['joint_target'],rtol=0,atol=1e-7)]
        receipts=[w for w in rows if w.get('kind')=='token_received' and w['time']<=r['time'] and r['time']-w['time']<=1 and np.asarray(w.get('token',[])).shape==np.asarray(r.get('token',[])).shape and np.allclose(w['token'],r.get('token',[]),rtol=0,atol=1e-7)] if 'token' in r else []
        results.append(dict(synthetic=bool(r.get('synthetic',False)),dds_write_candidates=writes,token_receipt_candidates=receipts,time=r['time'],tick=r['control_tick'],raw_action=action.tolist(),mapped_target=target.tolist(),recorded_target=r['joint_target'],target_delta=delta.tolist(),largest_joint=int(np.argmax(np.abs(delta))),max_delta=float(np.max(np.abs(delta))),decoder_matches_recorded_trt=passed,max_decoder_error=float(np.max(np.abs(action-expected)))))
    a.output.write_text(json.dumps(dict(runtime='Original PolicyEngine TensorRT replay of captured full inputs; no hardware commands',rtol=1e-3,atol=1e-4,results=results),indent=2))
    if not all(r['decoder_matches_recorded_trt'] for r in results):raise SystemExit('Decoder parity FAILED; inspect report, do not claim replay equivalence')
if __name__=='__main__':main()
