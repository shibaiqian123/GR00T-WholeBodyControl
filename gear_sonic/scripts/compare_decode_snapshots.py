"""Offline counterfactual only. No DDS/ZMQ/robot startup imports or commands."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np

SCHEMAS = {'sonic_v1_1_prospective_control_v1': 'WAIT_FOR_CONTROL',
           'sonic_v1_1_planner_observation_v1': 'CONTROL_PLANNER'}

def vector(row, key, n):
    a = np.asarray(row[key], dtype=np.float64)
    if a.shape != (n,) or not np.isfinite(a).all():
        raise ValueError(f'{key}: requires {n} finite values')
    return a

def prepare(row, token):
    if SCHEMAS.get(row.get('schema_name')) != row.get('state'):
        raise ValueError('unsupported snapshot schema/state')
    x = vector(row, 'decoder_input', 994).copy()
    x[:64] = vector({'token': token}, 'token', 64)
    for key in ('actual_q', 'actual_dq', 'scale', 'default_angles'):
        vector(row, key, 29)
    mapping = vector(row, 'mapping', 29)
    if sorted(mapping.tolist()) != list(range(29)):
        raise ValueError('mapping must be permutation 0..28')
    if not np.isfinite(float(row['sample_time'])):
        raise ValueError('missing sample time')
    return x.astype('<f4')

def targets(raw, row):
    raw = vector({'raw': raw}, 'raw', 29)
    # Hardware order, matching CreatePolicyCommand, including final float32 cast.
    return (raw[np.asarray(row['mapping'], dtype=int)] * vector(row, 'scale', 29)
            + vector(row, 'default_angles', 29)).astype(np.float32)

def sha(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1024*1024), b''): h.update(chunk)
    return h.hexdigest()

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--wait-snapshot', type=Path, required=True, help='one exact JSON snapshot row')
    p.add_argument('--planner-snapshot', type=Path, required=True)
    p.add_argument('--token', type=Path, required=True, help='64-value .npy file')
    p.add_argument('--decoder', type=Path, required=True, help='standalone replay_sonic_decoder executable')
    p.add_argument('--model', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    rows = [json.loads(f.read_text()) for f in (a.wait_snapshot, a.planner_snapshot)]
    if [r.get('state') for r in rows] != ['WAIT_FOR_CONTROL', 'CONTROL_PLANNER']:
        raise ValueError('requires WAIT and PLANNER snapshots, in that order')
    token = np.load(a.token, allow_pickle=False)
    inputs = np.stack([prepare(r, token) for r in rows])
    a.output.mkdir(parents=True, exist_ok=False)
    inp, out = a.output/'inputs.f32', a.output/'raw_actions.f32'
    inputs.tofile(inp)
    provenance = {str(f.resolve()): sha(f) for f in (a.wait_snapshot, a.planner_snapshot, a.token, a.decoder, a.model)}
    (a.output/'provenance.json').write_text(json.dumps(provenance, indent=2))
    with (a.output/'decoder.log').open('w') as log:
        subprocess.run([str(a.decoder.resolve()), str(a.model.resolve()), str(inp.resolve()), str(out.resolve())], stdout=log, stderr=subprocess.STDOUT, check=True, timeout=120)
    raw = np.fromfile(out, dtype='<f4')
    if raw.size != 58 or not np.isfinite(raw).all(): raise ValueError('invalid decoder output')
    meta = json.loads(Path(str(out)+'.mapping.json').read_text())
    for r in rows:
        for k in ('mapping', 'scale', 'default_angles'):
            if not np.allclose(vector(r,k,29),vector(meta,k,29),rtol=0,atol=1e-12):
                raise ValueError(f'decoder resource mapping mismatch: {k}')
    result = {'motor_written': False, 'executed': False,
              'interpretation': 'counterfactual same-token decode; not a simulated mode transition or hardware acceptance', 'samples': []}
    for r, y in zip(rows, raw.reshape(2,29)):
        q = targets(y,r).astype(float); d = q-vector(r,'actual_q',29); j = int(np.argmax(abs(d)))
        result['samples'].append(dict(state=r['state'], sample_time=r['sample_time'],
            stability='not_assessed', padding_rows=r.get('padding_rows', 'not_recorded'),
            actual_q=r['actual_q'], actual_dq=r['actual_dq'], raw_action=y.tolist(),
            joint_target=q.tolist(), delta=d.tolist(), max_joint_index=j, max_abs_delta=float(abs(d[j]))))
    (a.output/'result.json').write_text(json.dumps(result, indent=2, allow_nan=False))
    print(a.output/'result.json')

if __name__ == '__main__': main()
