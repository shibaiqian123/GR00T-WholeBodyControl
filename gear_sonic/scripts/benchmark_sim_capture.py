"""Offline snapshot/serialization/write/physics costs. No viewer or DDS."""
import json,time,tempfile
from pathlib import Path
import numpy as np
import mujoco
from gear_sonic.utils.mujoco_sim.startup_capture import encode
root=Path('outputs/sim_startup_audit_writerfix_round1/1_model_only_20261028_203518')
rows=[]
with (root/'sim.jsonl').open() as f:
 for line in f:
  r=json.loads(line)
  if r.get('kind')=='sim_state':rows.append(r)
  if len(rows)>=1000:break
model=mujoco.MjModel.from_binary_path(str(root/'sim.mjb'));data=mujoco.MjData(model)
measure={k:[] for k in ('copy','encode','write','physics')}
with tempfile.TemporaryFile(mode='w',dir=root) as f:
 for r in rows:
  for k in ('body_q','body_dq','qpos','qvel'):r[k]=np.asarray(r[k])
  for _ in range(3):
   start=time.perf_counter();snapshot=dict(r)
   for k in ('body_q','body_dq','qpos','qvel'):snapshot[k]=r[k].copy()
   after=time.perf_counter();line=json.dumps(snapshot,default=encode,allow_nan=False)+'\n';encoded=time.perf_counter();f.write(line);end=time.perf_counter()
   measure['copy'].append(after-start);measure['encode'].append(encoded-after);measure['write'].append(end-encoded)
  data.qpos[:]=r['qpos'];data.qvel[:]=r['qvel'];mujoco.mj_forward(model,data)
  start=time.perf_counter();mujoco.mj_step(model,data);measure['physics'].append(time.perf_counter()-start)
result={k:dict(n=len(v),mean_ms=float(np.mean(v)*1000),p99_ms=float(np.quantile(v,.99)*1000),max_ms=float(max(v)*1000)) for k,v in measure.items()}
result['limitations']='Offline isolated costs; physics lacks captured motor targets/support force. No live renderer, DDS, full-loop scheduling or contention. Does not establish root cause.'
(root.parent/'offline_costs.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
