"""Opt-in simulation state capture, bounded asynchronous output; never controls DDS."""
import atexit
import hashlib
import json
import os
from pathlib import Path
import queue
import threading
import time

_writer = None

def encode(value):
    if hasattr(value, 'tolist'): return value.tolist()
    if isinstance(value, Path): return str(value)
    raise TypeError(type(value).__name__)

class Capture:
    def __init__(self, env):
        if env.config['DOMAIN_ID'] != 38 or env.config['INTERFACE'] != 'lo':
            raise RuntimeError('Simulation audit requires DDS domain 38 on lo')
        self.path = Path(os.environ['SONIC_SIM_AUDIT'])
        self.file = self.path.open('x')
        self.queue = queue.Queue(2048)
        self.dropped = 0
        self.error = None
        self.closed = False
        import mujoco
        xml = self.path.with_suffix('.xml')
        mujoco.mj_saveLastXML(str(xml), env.mj_model)
        mjb = self.path.with_suffix('.mjb')
        mujoco.mj_saveModel(env.mj_model, str(mjb))
        self.file.write(json.dumps(dict(kind='sim_configuration', config=env.config,
            model_xml_sha256=hashlib.sha256(xml.read_bytes()).hexdigest(),
            model_mjb_sha256=hashlib.sha256(mjb.read_bytes()).hexdigest(),
            joint_names=[env.mj_model.joint(i).name for i in range(env.mj_model.njnt)],
            body_joint_index=env.body_joint_index,
            note='Default simulator initial state, not reconstructed real initial state'), default=encode)+'\n')
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()
    def run(self):
        try:
            while True:
                row=self.queue.get()
                if row is None: break
                if os.environ.get('SONIC_SIM_TIMING'):
                    from gear_sonic.utils.mujoco_sim.loop_timing import record
                    start=time.monotonic()
                    line=json.dumps(row, default=encode, allow_nan=False)+'\n'
                    encoded=time.monotonic()
                    self.file.write(line)
                    record('capture_encode',start,encoded)
                    record('capture_write',encoded,time.monotonic())
                else:
                    self.file.write(json.dumps(row, default=encode, allow_nan=False)+'\n')
        except Exception as exc: self.error=repr(exc)
    def put(self, env):
        band = getattr(env, 'elastic_band', None)
        support = dict(enabled=bool(band and band.enable), floating_root=env.use_floating_root_link,
            anchor=band.point.copy() if band else None, length=band.length if band else None,
            applied_force=env.mj_data.xfrc_applied[env.band_attached_link].copy() if band else None)
        row=dict(support=support, kind='sim_state', time=time.monotonic(), sim_time=float(env.mj_data.time),
            body_q=env.obs['body_q'].copy(), body_dq=env.obs['body_dq'].copy(),
            qpos=env.mj_data.qpos.copy(), qvel=env.mj_data.qvel.copy())
        try: self.queue.put_nowait(row)
        except queue.Full: self.dropped+=1
    def close(self):
        if self.closed: return
        self.closed=True
        if self.thread.is_alive(): self.queue.put(None)
        self.thread.join()
        self.file.write(json.dumps(dict(kind='sim_audit_end', dropped=self.dropped,error=self.error))+'\n')
        self.file.close()

def prepare(env):
    """Save model resources and start writer before realtime deadlines begin."""
    global _writer
    if _writer is None:
        _writer=Capture(env)
        atexit.register(close)

def capture(env):
    prepare(env)
    _writer.put(env)

def close():
    if _writer is not None: _writer.close()
