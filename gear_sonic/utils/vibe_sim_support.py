"""Simulation-only helpers; original SONIC band algorithm is loaded without DDS imports."""
import ast, hashlib, re
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]

def original_band():
    import scipy.spatial.transform
    path=ROOT/"gear_sonic/utils/mujoco_sim/unitree_sdk2py_bridge.py"
    source=path.read_text()
    node=next(n for n in ast.parse(source).body if isinstance(n,ast.ClassDef) and n.name=="ElasticBand")
    namespace={"np":np,"scipy":__import__("scipy")}
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),"exec"),namespace)
    return namespace["ElasticBand"](),hashlib.sha256(source.encode()).hexdigest()

def validate_standing_angles(case):
    path=ROOT/"gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/policy_parameters.hpp"
    text=path.read_text().split("default_angles = {",1)[1].split("};",1)[0]
    text=re.sub(r"//[^\n]*","",text)
    values=np.array([float(v.strip()) for v in text.split(",") if v.strip()])
    if values.shape!=(29,) or not np.allclose(values,case.default,rtol=0,atol=1e-7):
        raise ValueError("SONIC default stance differs from training joint defaults")
    return values

def standing_reset(case):
    """Find a flat, collision-free patch near the seed; never modify the saved scene."""
    m,d,mj=case.model,case.data,case.mujoco
    if abs(float(m.opt.timestep)-.005)>1e-10:raise ValueError("Expected 5 ms physics")
    defaults=validate_standing_angles(case)
    case.reset()
    free=int(m.body_jntadr[case.root])
    if free<0 or m.jnt_type[free]!=mj.mjtJoint.mjJNT_FREE:raise ValueError("Missing root free joint")
    qa=int(m.jnt_qposadr[free])
    center=d.qpos[qa:qa+2].copy()
    heading=float(np.arctan2(d.xmat[case.root].reshape(3,3)[1,0],d.xmat[case.root].reshape(3,3)[0,0]))
    upright_quat=np.array([np.cos(heading/2),0.,0.,np.sin(heading/2)])
    robot_bodies=set()
    for b in range(m.nbody):
        j=b
        while j>0 and j!=case.root:j=int(m.body_parentid[j])
        if j==case.root:robot_bodies.add(b)
    robot_geoms=[g for g in range(m.ngeom) if int(m.geom_bodyid[g]) in robot_bodies]
    feet=[g for g in robot_geoms if "foot" in (mj.mj_id2name(m,mj.mjtObj.mjOBJ_GEOM,g) or "")
          and (m.geom_contype[g] or m.geom_conaffinity[g])]
    if not feet or any(m.geom_type[g]!=mj.mjtGeom.mjGEOM_CAPSULE for g in feet):
        raise ValueError("Expected training foot capsule collisions")
    groups=m.geom_group.copy()
    candidates=[]
    # Ray cast only collision geometry outside the robot; restore render groups immediately.
    try:
        m.geom_group[:]=5
        for g in range(m.ngeom):
            if g not in robot_geoms and (m.geom_contype[g] or m.geom_conaffinity[g]):m.geom_group[g]=4
        mask=np.array([0,0,0,0,1,0],np.uint8)
        for dx,dy in sorted(((x,y) for x in (0.,-.5,.5,-1.,1.,-1.5,1.5,-2.,2.)
                                      for y in (0.,-.5,.5,-1.,1.,-1.5,1.5,-2.,2.)),key=lambda v:v[0]**2+v[1]**2):
            xy=center+np.array([dx,dy]);heights=[]
            for ox in (-.3,0.,.3):
                for oy in (-.3,0.,.3):
                    gid=np.array([-1],np.int32)
                    distance=mj.mj_ray(m,d,np.array([xy[0]+ox,xy[1]+oy,10.]),np.array([0.,0.,-1.]),mask,1,-1,gid)
                    heights.append(10.-distance if distance>=0 else np.nan)
            if np.isfinite(heights).all() and np.ptp(heights)<1e-4:
                candidates.append((float(np.mean(heights)),float(dx*dx+dy*dy),xy))
    finally:m.geom_group[:]=groups
    if not candidates:raise ValueError("No verified flat 0.6 m patch near seed")
    # Prefer lowest floor, then closest point; this excludes stair tops when a floor is present.
    for floor,_,xy in sorted(candidates,key=lambda v:(round(v[0],4),v[1])):
        d.qpos[case.qids]=defaults;d.qpos[qa:qa+3]=[xy[0],xy[1],1.]
        d.qpos[qa+3:qa+7]=upright_quat;d.qvel[:]=0;d.ctrl[case.aids]=defaults
        mj.mj_forward(m,d)
        foot_min=min(d.geom_xpos[g,2]-abs(d.geom_xmat[g].reshape(3,3)[2,2])*m.geom_size[g,1]-m.geom_size[g,0] for g in feet)
        d.qpos[qa+2]+=floor+.002-foot_min
        mj.mj_forward(m,d)
        bad=[float(d.contact[i].dist) for i in range(d.ncon) if d.contact[i].dist < -1e-5]
        if bad:continue
        case.last=np.zeros(29,np.float32)
        from gear_sonic.utils.standalone.native import History
        case.history=History(case.terms())
        return {"mode":"standing","root_position":d.xpos[case.root].tolist(),
                "root_quaternion":d.xquat[case.root].tolist(),"heading_radians":heading,"floor_height":floor,
                "minimum_contact_distance":min([float(d.contact[i].dist) for i in range(d.ncon)] or [0.]),
                "foot_clearance":.002,"flat_patch_width":.6}
    raise ValueError("All flat standing candidates collide; no tolerance relaxation")

def apply_band(case,band):
    mj,m,d=case.mujoco,case.model,case.data
    if not band.enable:
        d.xfrc_applied[case.root]=0
        return
    velocity=np.zeros(6)
    mj.mj_objectVelocity(m,d,mj.mjtObj.mjOBJ_BODY,case.root,velocity,0)
    # Express angular error in the initial heading frame; retain the original spring law.
    # Translation/force remain world-frame. This preserves yaw without changing gains.
    from scipy.spatial.transform import Rotation
    heading=Rotation.from_euler("z",getattr(band,"reference_heading",0.))
    quat=d.xquat[case.root]
    relative=(heading.inv()*Rotation.from_quat(quat[[1,2,3,0]])).as_quat()
    pose=np.concatenate([d.xpos[case.root],relative[[3,0,1,2]],velocity[3:],heading.inv().apply(velocity[:3])])
    wrench=band.Advance(pose)
    wrench[3:]=heading.apply(wrench[3:])
    d.xfrc_applied[case.root]=wrench
