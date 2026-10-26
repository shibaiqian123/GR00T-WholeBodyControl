"""Strict live SMPL adapter for simulation-only hybrid deployment."""
from collections import deque
from types import SimpleNamespace
import numpy as np

class LiveWindow:
    """11 actual 50 Hz samples: derivative predecessor plus 10 future frames."""
    def __init__(self):
        self.rows = deque(maxlen=11)
        self.session = None
        self.scale = None

    def append(self, p):
        if p.get("schema") != "groot.vibe.pose.v1":
            raise ValueError("Wrong pose schema")
        if p["session"] != self.session:
            self.rows.clear()
            self.scale = None
            self.session = p["session"]
        stamp = int(p["sample_ns"])
        if self.rows:
            delta = stamp - self.rows[-1]["sample_ns"]
            if delta != 20_000_000:
                self.rows.clear()
                raise ValueError(f"50 Hz timestamp discontinuity: {delta}")
        r = {"sample_ns":stamp, "received_monotonic":float(p["received_monotonic"])}
        for key,shape in (("joints_local",(24,3)),("root_quat",(4,)),
                          ("root_translation_zup",(3,)),("wrist",(6,))):
            a = np.asarray(p[key],np.float32)
            if a.shape != shape or not np.isfinite(a).all():
                raise ValueError("Invalid "+key)
            r[key] = a
        if abs(np.linalg.norm(r["root_quat"])-1)>1e-4:
            raise ValueError("Nonunit root quaternion")
        self.rows.append(r)

    def observations(self, robot_quat, robot_points, robot_names, now, heading=None):
        if len(self.rows)!=11:
            raise ValueError(f"PICO warming up: {len(self.rows)}/11")
        if not 0 <= now-self.rows[-1]["received_monotonic"] <= .25:
            raise ValueError("PICO timestamp stale or in future")
        import torch
        from gear_sonic.utils.standalone.math_ops import quat_apply, quat_apply_inverse, quat_mul, quat_conjugate, axis_angle_from_quat
        from gear_sonic.utils.standalone.preprocess import sonic_smpl_tokenizer
        from gear_sonic.utils.standalone.preprocess import IL2MJ
        from gear_sonic.utils.standalone.preprocess import infer_morphology_scale, scaled_smpl_targets
        rows=list(self.rows)
        q=torch.tensor(np.stack([r["root_quat"] for r in rows]))
        local=torch.tensor(np.stack([r["joints_local"] for r in rows]))
        trans=torch.tensor(np.stack([r["root_translation_zup"] for r in rows]))
        if heading is not None:
            h=torch.tensor(heading,dtype=torch.float32).expand(11,-1)
            if h.shape!=(11,4) or not torch.isfinite(h).all() or abs(float(torch.linalg.vector_norm(h[0]))-1)>1e-4:
                raise ValueError("Invalid SONIC heading correction")
            q=quat_mul(h,q)
            trans=quat_apply(h,trans)
        raw=quat_apply(q[:,None,:].expand(-1,24,-1),local)
        world=raw+trans[:,None,:]
        if self.scale is None:
            self.scale=infer_morphology_scale(world,torch.tensor(robot_points),robot_names)
        mapped=scaled_smpl_targets(world,self.scale,torch.zeros(3))
        velocity=(mapped[1,0]-mapped[0,0])/.02
        delta=quat_mul(q[1],quat_conjugate(q[0]))
        angular=axis_angle_from_quat(delta)/.02
        query=torch.cat((quat_apply_inverse(q[1],velocity),quat_apply_inverse(q[1],angular)))
        il=torch.zeros((11,29))
        il[:,23:29]=torch.tensor(np.stack([r["wrist"] for r in rows]))
        motion=SimpleNamespace(smpl_joints=raw,smpl_root_quat=q,joint_pos=il[:,IL2MJ])
        command=SimpleNamespace(motion=motion,future_frames=lambda n,s:torch.arange(1,1+n*s,s)[None])
        robot=SimpleNamespace(data=SimpleNamespace(root_link_quat_w=torch.tensor(robot_quat,dtype=torch.float32)[None]))
        env=SimpleNamespace(num_envs=1,scene={"robot":robot},command_manager=SimpleNamespace(get_term=lambda _:command))
        return sonic_smpl_tokenizer(env).numpy(),query.numpy()[None],rows[1]["sample_ns"]

class VisualModels:
    """Reuse validated ViBe export wrappers and SONIC's FP32 TensorRT library."""
    def __init__(self,cfg,library,cache,out):
        import json, importlib.util
        from pathlib import Path
        import onnxruntime as ort
        from gear_sonic.utils.standalone import trt_session as trt
        from gear_sonic.utils.vibe_config import digest
        root=Path(cfg["bundle"]).parent
        manifest=json.loads((root/"visual_decoder.manifest.json").read_text())
        encoder_manifest=json.loads((root/"motion_encoder.manifest.json").read_text())
        self.agent=SimpleNamespace(manifest=manifest,encoder_port=encoder_manifest["inputs"][0])
        if cfg["schema"]!="groot.vibe.standalone.v1":
            raise ValueError("Prepare a standalone deployment config first")
        adapted=Path(cfg["vision_trt"])
        if digest(adapted)!=cfg["vision_trt_sha256"]:raise ValueError("Prepared vision changed")
        root=Path(cfg["bundle"]).parent;alias=cfg.get("model_paths",{})
        paths=[Path(alias.get("motion_encoder",root/"motion_encoder.onnx")),
               Path(alias.get("visual_decoder",root/"visual_decoder.onnx")),Path(cfg["vision_encoder"])]
        self.engines=[trt.Session(library,p,cache) for p in (paths[0],paths[1],adapted)]
        opt=ort.SessionOptions();opt.intra_op_num_threads=1;opt.inter_op_num_threads=1
        self.refs=[ort.InferenceSession(str(p),sess_options=opt,providers=["CPUExecutionProvider"]) for p in paths]
        for session,expected in zip(self.refs[:2],(encoder_manifest,manifest)):
            if json.loads(session.get_modelmeta().custom_metadata_map["manifest"])!=expected:
                raise ValueError("Embedded model manifest differs")
        self.vision=json.loads(paths[2].with_suffix(".manifest.json").read_text())
        if self.vision!=json.loads(self.refs[2].get_modelmeta().custom_metadata_map["manifest"]):
            raise ValueError("Embedded Theia manifest differs")

    def infer(self,obs,rgb,baseline=False):
        from gear_sonic.utils.standalone.native import bgr_input
        def run(i,feed):
            if baseline:
                return dict(zip([p.name for p in self.refs[i].get_outputs()],self.refs[i].run(None,feed)))
            if i==2:feed={k:v.astype(np.float32) for k,v in feed.items()}
            return self.engines[i].run(feed)
        latent=run(0,{self.agent.encoder_port["name"]:obs["tokenizer"]})["latent"]
        features=run(2,{self.vision["input"]["name"]:bgr_input(rgb)})
        feed=dict(obs,latent=latent)
        for p in self.agent.manifest["inputs"]:
            for term in p["terms"]:
                source=term.get("source",{})
                if source.get("kind")=="vision_encoder":
                    feed[p["name"]]=features["patch_tokens" if source["slice"]=="tokens" else "cls_token"]
        feed={p.name:feed[p.name] for p in self.refs[1].get_inputs()}
        return latent,run(1,feed)["actions"]

    def close(self):
        for engine in self.engines:engine.close()

HARDWARE_JOINTS = tuple(
    [side+"_"+joint+"_joint" for side in ("left","right")
     for joint in ("hip_pitch","hip_roll","hip_yaw","knee","ankle_pitch","ankle_roll")]
    + ["waist_"+joint+"_joint" for joint in ("yaw","roll","pitch")]
    + [side+"_"+joint+"_joint" for side in ("left","right")
       for joint in ("shoulder_pitch","shoulder_roll","shoulder_yaw","elbow","wrist_roll","wrist_pitch","wrist_yaw")]
)

def validate_hardware_order(cfg):
    if tuple(cfg["action"]["joint_names"]) != HARDWARE_JOINTS:
        raise ValueError("Export order differs from the explicit SONIC DDS motor order")
