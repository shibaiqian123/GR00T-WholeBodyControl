"""Analyze partial or complete simulation traces without hiding capture failures."""
import argparse,json,subprocess
from pathlib import Path
import numpy as np

def read(path):
    return [json.loads(s) for s in path.read_text().splitlines()] if path.exists() else []

def analyze(out,engine,decoder):
    result=json.loads((out/'results.json').read_text());summary=[]
    for case in result:
        root=Path(case['output']);rows=read(root/'bottom.jsonl');events=read(root/'replay.jsonl');sim=read(root/'sim.jsonl')
        commands=[r for r in rows if r.get('kind')=='decoder_command']
        sends=[r for r in events if r['kind']=='send' and r['source']['kind']=='frame']
        receipts=[r for r in rows if r.get('kind')=='token_received']
        item=dict(case=case['case'],capture_passed=case['passed'],failure=case['failure'],decoder_count=len(commands),sent=len(sends),received_records=len(receipts),
            audit_footer=rows[-1] if rows else None)
        if len(sends)>1:
            source=np.diff([r['source']['time'] for r in sends]);actual=np.diff([r['sent'] for r in sends])
            item['send_timing']=dict(min_seconds=float(actual.min()),median_seconds=float(np.median(actual)),max_seconds=float(actual.max()),max_compression_seconds=float(np.max(source-actual)),max_lateness_seconds=max(r['lateness'] for r in sends),note='Send intervals only; missing receipt logs prevent proving complete receiver queue behavior')
        matches=[]
        for r in receipts:
            candidates=[x for x in sends if np.allclose(x['source']['token'],r['token'],rtol=0,atol=1e-7) and x['sent']<=r['time']+.002]
            matches.append(dict(time=r['time'],send_candidates=[x['source']['index'] for x in candidates],age_candidates=[r['time']-x['sent'] for x in candidates]))
        item['receipt_matches']=matches
        if commands:
            first=commands[0];q=np.asarray(first['actual_q']);target=np.asarray(first['joint_target']);delta=target-q
            token_candidates=[x for x in sends if np.allclose(x['source']['token'],first['token'],atol=1e-7,rtol=0) and x['sent']<=first['time']]
            src_events=read(Path(json.loads((out/'manifest.json').read_text())['cases'][len(summary)]['source']))
            prediction_matches=[]
            for x in token_candidates:
                for pred in src_events:
                    if pred.get('kind')!='prediction' or not pred.get('ok') or pred['time']>x['source']['time']:continue
                    values=np.asarray(pred.get('actions',[]))
                    if values.shape!=(50,66):continue
                    for j in np.flatnonzero(np.all(np.isclose(values[:,:64],first['token'],atol=1e-7,rtol=0),axis=1)):
                        prediction_matches.append(dict(request_id=pred['request_id'],window_index=int(j),source_age=x['source']['time']-pred['observed_at'],send_counter=x['source']['index']))
            item['first']=dict(time=first['time'],tick=first['control_tick'],using_encoder=first.get('using_encoder'),reference=first.get('reference_motion'),actual_q=q.tolist(),target=target.tolist(),delta=delta.tolist(),largest_joint=int(np.argmax(abs(delta))),max_delta_rad=float(max(abs(delta))),source_candidates=prediction_matches)
            obs=np.asarray(first['decoder_input']);offset=64;history={}
            for name,width in [('angular_velocity',3),('joint_positions',29),('joint_velocities',29),('last_actions',29),('gravity',3)]:
                a=obs[offset:offset+10*width].reshape(10,width);offset+=10*width
                history[name]=dict(zero_rows=np.flatnonzero(np.all(a==0,axis=1)).tolist(),rows=a.tolist())
            item['first_history']=history
            item['events_before_first']=[r for r in rows if r.get('kind') not in ('dds_write_attempt','waiting_state','decoder_command','token_received') and first['time']-2<=r.get('time',-1)<=first['time']]
            writes=[r for r in rows if r.get('kind')=='dds_write_attempt' and r['time']>=first['time'] and np.allclose(r['q'],target,atol=1e-7,rtol=0)]
            item['first_target_dds_attempt']=writes[0] if writes else None
            item['trajectory']=[dict(time=r['time']-first['time'],actual_q=r['actual_q'],target=r['joint_target']) for r in commands]
            prior=[r for r in sim if r.get('kind')=='sim_state' and r['time']<=first['time']]
            item['prior_sim_state']=prior[-1] if prior else None
            # Partial-input parity is deliberately separate from the strict full-trace verifier.
            inp=root/'captured_inputs.f32';output=root/'recomputed_actions.f32'
            np.asarray([r['decoder_input'] for r in commands],dtype=np.float32).tofile(inp)
            with (root/'partial_input_parity.log').open('w') as log:
                completed=subprocess.run([str(engine),str(decoder),str(inp),str(output)],stdout=log,stderr=subprocess.STDOUT)
            if completed.returncode==0:
                actions=np.fromfile(output,dtype=np.float32).reshape(-1,29);expected=np.asarray([r['raw_action'] for r in commands])
                item['partial_input_parity']=dict(passed=bool(np.allclose(actions,expected,rtol=1e-3,atol=1e-4)),max_error=float(np.max(abs(actions-expected))),rtol=1e-3,atol=1e-4,note='Captured complete rows only; does not repair dropped records or certify full trace')
            else:item['partial_input_parity']=dict(passed=False,error='engine replay failed')
        summary.append(item)
    (out/'analysis.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps([{k:v for k,v in x.items() if k in ('case','failure','capture_passed','decoder_count','sent','received_records','send_timing','first','partial_input_parity')} for x in summary],indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    root=Path(__file__).resolve().parents[2]/'gear_sonic_deploy'
    analyze(a.output,root/'target/release/replay_sonic_decoder',root/'policy/sonic_v1_1/model_decoder.onnx')
