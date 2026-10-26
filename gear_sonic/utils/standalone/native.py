"""Independent MuJoCo/ONNX loop; no training env.step, DDS, or external forces."""
from __future__ import annotations
import argparse
import hashlib
import json
import time
from pathlib import Path
from types import SimpleNamespace
import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class History:
    """Term-major, chronological history, matching mjlab CircularBuffer."""
    def __init__(self, values, length=10):
        self.values = [np.repeat(np.asarray(x, np.float32)[None], length, axis=0) for x in values]
    def append(self, values):
        for history, value in zip(self.values, values, strict=True):
            history[:-1] = history[1:]
            history[-1] = value
    def flat(self):
        return np.concatenate([x.ravel() for x in self.values])[None].astype(np.float32)


class Deadline:
    """SONIC diagnostic fix: cumulative deadlines, not sleep(dt) each iteration."""
    def __init__(self, dt):
        self.dt, self.next = dt, time.perf_counter()
    def wait(self):
        self.next += self.dt
        delay = self.next - time.perf_counter()
        if delay > 0: time.sleep(delay)
    def resume(self):
        self.next = time.perf_counter()


def bgr_input(rgb):
    if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[-1] != 3:
        raise ValueError('Expected HWC uint8 RGB')
    return np.ascontiguousarray(rgb[None, :, :, ::-1])


def resolve(model, kind, name):
    import mujoco
    # The compiled training scene namespaces robot names.
    found = [mujoco.mj_name2id(model, kind, n) for n in (name, 'robot/'+name)]
    found = set(x for x in found if x >= 0)
    if len(found) != 1: raise ValueError(f'Ambiguous/missing model name {name}: {found}')
    return found.pop()


class RGBPanel:
    """Standard-library GUI, keeping deployment environment dependencies unchanged."""
    def __init__(self):
        import tkinter as tk
        self.root = tk.Tk()
        self.root.title('Actual Theia RGB - native MuJoCo')
        self.text = tk.Label(self.root, text='Waiting for policy input')
        self.text.pack()
        self.label = tk.Label(self.root)
        self.label.pack()
        self.closed = False
        self.root.protocol('WM_DELETE_WINDOW', self.close)
    def pump(self):
        if not self.closed:
            self.root.update_idletasks()
            self.root.update()
    def show(self, rgb, frame, mode):
        if self.closed: return
        from PIL import Image, ImageTk
        self.photo = ImageTk.PhotoImage(Image.fromarray(rgb).resize((448,252)))
        self.label.configure(image=self.photo)
        self.text.configure(text=f'Frame {frame} | {mode} | actual RGB before exported BGR conversion')
        self.pump()
    def close(self):
        if not self.closed:
            self.closed = True
            self.root.destroy()


class NativeCase:
    def __init__(self, directory, manifest):
        import mujoco
        import torch
        from .preprocess import sonic_smpl_tokenizer
        self.mujoco, self.torch, self.tokenizer_fn = mujoco, torch, sonic_smpl_tokenizer
        directory = Path(directory)
        self.meta = json.loads((directory/'case.json').read_text())
        for filename, value in self.meta['files'].items():
            if sha(directory/filename) != value: raise ValueError(f'Changed case file: {filename}')
        self.initial = dict(np.load(directory/'initial.npz', allow_pickle=False))
        self.reference = dict(np.load(directory/'reference.npz', allow_pickle=False))
        self.model = mujoco.MjModel.from_binary_path(str(directory/'scene.mjb'))
        self.data = mujoco.MjData(self.model)
        names = manifest['action']['joint_names']
        if list(self.meta['joint_names']) != names: raise ValueError('Robot joint order differs from exported actor')
        self.jids = np.array([resolve(self.model,mujoco.mjtObj.mjOBJ_JOINT,n) for n in names])
        self.qids = self.model.jnt_qposadr[self.jids]
        self.vids = self.model.jnt_dofadr[self.jids]
        self.root = resolve(self.model,mujoco.mjtObj.mjOBJ_BODY,'pelvis')
        self.torso = resolve(self.model,mujoco.mjtObj.mjOBJ_BODY,'torso_link')
        self.cam = resolve(self.model,mujoco.mjtObj.mjOBJ_CAMERA,'head_cam')
        self.points = [resolve(self.model,mujoco.mjtObj.mjOBJ_BODY,n) for n in self.meta['point_body_names']]
        self.aids = []
        for j in self.jids:
            ids = np.flatnonzero((self.model.actuator_trnid[:,0] == j) & (self.model.actuator_trntype == 0))
            if len(ids) != 1: raise ValueError('Only one native position actuator per joint supported')
            self.aids.append(int(ids[0]))
        if len(self.aids) != self.model.nu: raise ValueError('Unmapped actuators')
        action = manifest['action']
        self.default = np.asarray(action['default_joint_pos'],np.float32)
        self.scale = np.asarray(action['scale'],np.float32)
        if not np.allclose(self.initial['default_joint_pos'],self.default,atol=1e-7):
            raise ValueError('Default joint pose mismatch')
        for i, aid in enumerate(self.aids):
            if not np.allclose(self.model.actuator_gainprm[aid,0],action['stiffness'][i],rtol=1e-6):
                raise ValueError('PD stiffness mismatch')
            if not np.allclose(self.model.actuator_biasprm[aid,1:3],[-action['stiffness'][i],-action['damping'][i]],rtol=1e-6):
                raise ValueError('Native actuator does not implement expected PD')
        motion = SimpleNamespace(**{k:torch.tensor(self.reference[k]) for k in ('smpl_joints','smpl_root_quat','joint_pos')})
        self.command = SimpleNamespace(motion=motion,future_frames=self.future_frames)
        self.robot = SimpleNamespace(data=SimpleNamespace())
        self.shim = SimpleNamespace(num_envs=1,scene={'robot':self.robot},command_manager=SimpleNamespace(get_term=lambda _:self.command))
        self.reset()
    def future_frames(self, steps, skip):
        return self.torch.tensor(np.minimum(self.frame+np.arange(steps)*skip,self.meta['frames']-1)[None])
    def reset(self):
        self.mujoco.mj_resetData(self.model,self.data)
        for name in ('qpos','qvel','ctrl'): getattr(self.data,name)[:] = self.initial[name]
        self.frame = self.meta['initial_frame']
        self.last = self.initial['last_action'].astype(np.float32).copy()
        self.mujoco.mj_forward(self.model,self.data)
        self.history = History(self.terms())
    def terms(self):
        # mj_objectVelocity(..., local=1) uses inertial axes for BODY.
        # Training explicitly rotates WORLD angular velocity into LINK axes.
        rot = self.data.xmat[self.root].reshape(3,3)
        return [rot.T @ self.data.cvel[self.root,:3],self.data.qpos[self.qids]-self.default,self.data.qvel[self.vids],self.last,rot.T @ np.array([0.,0.,-1.])]
    def observations(self):
        self.robot.data.root_link_quat_w = self.torch.tensor(self.data.xquat[self.root][None],dtype=self.torch.float32)
        with self.torch.no_grad(): tokenizer = self.tokenizer_fn(self.shim).numpy()
        av,jp,jv,_,gravity = self.terms()
        return {'tokenizer':tokenizer,'policy':self.history.flat(),
            'augmentation':self.reference['augmentation'][self.frame][None].astype(np.float32),
            'q_proprio':np.concatenate([gravity,av,jp,jv])[None].astype(np.float32)}
    def step(self,action):
        action = np.asarray(action,np.float32).reshape(29)
        if not np.isfinite(action).all(): raise FloatingPointError('Non-finite action')
        clip = self.meta['clip_actions']
        if clip is not None: action = np.clip(action,-clip,clip)
        target = self.default+self.scale*action
        self.data.ctrl[self.aids] = target
        for _ in range(4): self.mujoco.mj_step(self.model,self.data)
        self.mujoco.mj_forward(self.model,self.data)
        self.last = action.copy()
        self.frame += 1
        self.history.append(self.terms())
        return target

