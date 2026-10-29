"""Opt-in bounded in-memory timing; no DDS or robot commands."""
import atexit
from array import array
import json
import os
from pathlib import Path
import time

_active = None
class Timing:
    def __init__(self, path, limit=100000):
        self.path=Path(path)
        # Fail before starting if output cannot be reserved; never overwrite evidence.
        self.file=self.path.open('x')
        self.limit=limit;self.samples={};self.dropped={};self.closed=False
    def add(self, name, start, end):
        rows=self.samples.setdefault(name,array('d'))
        if len(rows)//2>=self.limit:
            self.dropped[name]=self.dropped.get(name,0)+1;return
        rows.extend((start,end-start))
    def wrap(self, name, func):
        def call(*args,**kwargs):
            start=time.monotonic()
            try:return func(*args,**kwargs)
            finally:self.add(name,start,time.monotonic())
        return call
    def close(self):
        if self.closed:return
        self.closed=True
        json.dump(dict(schema=1,note='Wall time includes scheduling and GIL waits; nested stages overlap, do not sum',dropped=self.dropped,samples={k:list(v) for k,v in self.samples.items()}),self.file)
        self.file.close()

def install(simulator):
    global _active
    path=os.environ.get('SONIC_SIM_TIMING')
    if not path:return
    if _active:return
    env=simulator.sim_env
    if env.config['DOMAIN_ID']!=38 or env.config['INTERFACE']!='lo':
        raise RuntimeError('Timing diagnostics require domain 38 on lo')
    t=_active=Timing(path)
    for name in ('sim_step','update_viewer','update_reward','update_render_caches'):
        setattr(env,name,t.wrap(name,getattr(env,name)))
    import mujoco
    mujoco.mj_step=t.wrap('physics',mujoco.mj_step)
    from gear_sonic.utils.mujoco_sim import startup_capture
    startup_capture.capture=t.wrap('capture_snapshot',startup_capture.capture)
    atexit.register(t.close)

def record(name,start,end):
    if _active:_active.add(name,start,end)

def close():
    if _active:_active.close()
