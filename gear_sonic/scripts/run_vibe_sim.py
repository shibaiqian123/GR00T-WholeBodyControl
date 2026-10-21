"""GR00T opt-in recorded SMPL visual simulation. No DDS, planner, or real robot output."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from gear_sonic.utils.vibe_config import generate, load, digest


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def main():
    p = argparse.ArgumentParser(__doc__)
    p.add_argument('--policy-type', choices=['vibe'], required=True)
    p.add_argument('--bundle', required=True)
    p.add_argument('--vision-encoder', required=True)
    p.add_argument('--obs-config', required=True)
    p.add_argument('--prepare-config', action='store_true')
    p.add_argument('--case')
    p.add_argument('--vibe-root')
    p.add_argument('--input-type', choices=['recorded_smpl'], default='recorded_smpl')
    p.add_argument('--backend', choices=['trt', 'ort'], default='trt')
    p.add_argument('--trt-library')
    p.add_argument('--out')
    p.add_argument('--engine-cache', help='External reusable TensorRT cache; models remain hashed')
    p.add_argument('--check-only', action='store_true')
    p.add_argument('--steps', type=int, default=10000)
    p.add_argument('--viewer', action='store_true')
    p.add_argument('--hold-final', action='store_true')
    p.add_argument('--realtime', action='store_true')
    p.add_argument('--rgb-mode', choices=['normal','gray'], default='normal')
    p.add_argument('target', choices=['sim'])
    a = p.parse_args()
    if a.prepare_config:
        if not a.case or not a.vibe_root:
            p.error('--prepare-config requires --case and --vibe-root')
        cfg = generate(a.bundle, a.vision_encoder, a.case, a.vibe_root)
        dest = Path(a.obs_config); dest.parent.mkdir(parents=True, exist_ok=True)
        with dest.open('x') as f:
            json.dump(cfg, f, indent=2)
        print('CONFIG_GENERATED (not a closed-loop validation):', dest)
        return
    if not a.out or a.steps < 1:
        p.error('a fresh --out directory and positive --steps are required')
    if a.hold_final and not a.viewer:
        p.error('--hold-final requires --viewer')
    cfg = load(a.obs_config)
    supplied = Path(a.bundle).resolve()
    if supplied.is_dir(): supplied /= 'bundle.json'
    if str(supplied) != cfg['bundle'] or str(Path(a.vision_encoder).resolve()) != cfg['vision_encoder']:
        p.error('CLI model paths disagree with observation config')
    sys.path.insert(0, str(Path(cfg['vibe_root']) / 'src'))
    import numpy as np
    import mujoco
    import onnxruntime as ort
    from vibe.sim2sim.runtime import NativeCase, bgr_input, RGBPanel, Deadline
    from vibe.export.agent.split_sonic import SplitOnnxAgent
    from vibe.export.agent.run_split import backbone_id
    out = Path(a.out); out.mkdir(parents=True, exist_ok=False)
    report = {'status':'blocked', 'backend':a.backend, 'mode':a.rgb_mode, 'input_type':'recorded_smpl',
              'simulation_only':True, 'transport':'in_process_no_DDS', 'steps':0,
              'config_sha256':digest(a.obs_config), 'models':cfg['hashes']}
    renderer = viewer = panel = None
    engines = []
    rows = []
    active_wall = 0.0
    started = time.perf_counter()
    try:
        bundle_dir = Path(cfg['bundle']).parent
        agent = SplitOnnxAgent(bundle_dir, provider='cpu')
        case = NativeCase(cfg['case'], agent.manifest)
        paths = [bundle_dir/'motion_encoder.onnx', bundle_dir/'visual_decoder.onnx', Path(cfg['vision_encoder'])]
        opt = ort.SessionOptions(); opt.intra_op_num_threads=1; opt.inter_op_num_threads=1
        refs = [ort.InferenceSession(str(x),sess_options=opt,providers=['CPUExecutionProvider']) for x in paths]
        vm=json.loads(refs[2].get_modelmeta().custom_metadata_map['manifest'])
        if vm != json.loads(paths[2].with_suffix('.manifest.json').read_text()):
            raise ValueError('Embedded Theia manifest differs from sidecar')
        if a.backend=='trt':
            if not a.trt_library: raise ValueError('--trt-library is required for TensorRT')
            helpers = Path(cfg['vibe_root'])/'deploy/controller'
            trt=module('groot_vibe_trt',helpers/'trt_session.py')
            prepare=module('groot_vibe_vision',helpers/'prepare_vision_trt.py')
            adapted=out/'vision_float.onnx'; prepare.prepare(paths[2],adapted)
            for path in [*paths[:2],adapted]:
                engines.append(trt.Session(a.trt_library,path,Path(a.engine_cache) if a.engine_cache else Path(a.obs_config).parent/'engine_cache'))
            report['trt_library_sha256']=digest(a.trt_library)
        def run(index, feeds, baseline=False):
            if a.backend=='trt' and not baseline:
                if index==2: feeds={k:v.astype(np.float32) for k,v in feeds.items()}
                return engines[index].run(feeds)
            return dict(zip([x.name for x in refs[index].get_outputs()],refs[index].run(None,feeds),strict=True))
        def infer(obs,rgb,baseline=False):
            latent=run(0,{agent.encoder_port['name']:obs['tokenizer']},baseline)['latent']
            features=run(2,{vm['input']['name']:bgr_input(rgb)},baseline)
            feed={**obs,'latent':latent}
            for port in agent.manifest['inputs']:
                for term in port['terms']:
                    src=term.get('source',{})
                    if src.get('kind')=='vision_encoder':
                        if backbone_id(src['model_name'])!=vm['hf_id']:raise ValueError('Backbone mismatch')
                        feed[port['name']]=features['patch_tokens' if src['slice']=='tokens' else 'cls_token']
            feed={x.name:feed[x.name] for x in refs[1].get_inputs()}
            if not all(np.isfinite(x).all() for x in feed.values()):raise ValueError('Nonfinite inputs')
            action=run(1,feed,baseline)['actions']
            if not np.isfinite(action).all():raise ValueError('Nonfinite action')
            return latent,action
        obs=case.observations()
        report['observation_errors']={k:float(np.max(np.abs(v-case.initial['oracle__'+k]))) for k,v in obs.items()}
        if max(report['observation_errors'].values())>1e-5:raise ValueError('Observation parity failed')
        report['parity']=[]
        oracle={k:case.initial['oracle__'+k] for k in obs}
        for mode in ('normal','gray'):
            image=case.initial['rgb'][0].copy()
            if mode=='gray':image[:]=127
            l0,y0=infer(oracle,image,True);l1,y1=infer(obs,image)
            row={'mode':mode,'latent_exact':bool(np.array_equal(l0,l1)),
                 'action_max_abs':float(np.max(np.abs(y0-y1)))}
            report['parity'].append(row)
            if not row['latent_exact'] or row['action_max_abs']>1e-5:raise ValueError('Model parity failed')
        if a.check_only:
            report['status']='same_state_passed'; print('SAME_STATE_PASSED');return
        camera=cfg['camera']
        renderer=mujoco.Renderer(case.model,height=camera['height'],width=camera['width'])
        option=mujoco.MjvOption();option.geomgroup[:]=0;option.geomgroup[camera['geom_groups']]=1
        requests=[];paused=True
        if a.viewer:
            import mujoco.viewer
            viewer=mujoco.viewer.launch_passive(case.model,case.data,key_callback=requests.append)
            viewer.cam.lookat[:]=case.data.xpos[case.root];viewer.cam.distance=3
            panel=RGBPanel()
        else:paused=False
        print('VIBE RECORDED SIMULATION | SPACE=start/pause, R=reset | no PICO/DDS or real output',flush=True)
        deadline=Deadline(.02);last_display=0.; episode=0
        report['status']='stopped_by_user'
        for _ in range(a.steps):
            while paused or requests:
                while requests:
                    key=requests.pop(0)
                    if key==32:paused=not paused;deadline.resume()
                    if key in (82,114):case.reset();episode+=1;paused=True;deadline.resume()
                if viewer:
                    if not viewer.is_running():break
                    viewer.sync();panel.pump()
                if not paused:break
                time.sleep(.02)
            if viewer and not viewer.is_running():break
            if case.frame>=case.meta['frames']-1:
                report['status']='reference_completed_not_quality_certified';break
            t=time.perf_counter();frame=case.frame
            renderer.update_scene(case.data,camera=case.cam,scene_option=option)
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=camera['shadows']
            if not camera['textures']:
                for g in renderer.scene.geoms[:renderer.scene.ngeom]:g.texid=-1
            rgb=renderer.render().copy()
            if a.rgb_mode=='gray':rgb[:]=127
            _,act=infer(case.observations(),rgb);target=case.step(act)
            err=np.linalg.norm(case.data.xpos[case.points]-case.reference['point_targets'][case.frame],axis=-1)
            row={'episode':episode,'frame':frame,'sim_time':float(case.data.time),
                 'compute_s':time.perf_counter()-t,'root_up':float(case.data.xmat[case.root].reshape(3,3)[2,2]),
                 'tracking_mean':float(err.mean()),'tracking_max':float(err.max()),
                 'torque_saturation_fraction':float(np.mean(np.abs(case.data.actuator_force[case.aids]) >= np.max(np.abs(case.model.actuator_forcerange[case.aids]),axis=1)*.99)),
                 'min_contact_dist':float(min((x.dist for x in case.data.contact[:case.data.ncon]),default=0)),
                 'action':act.tolist(),'target':target.tolist()}
            rows.append(row)
            if len(rows)==1 or frame in (228,286,436):
                from PIL import Image
                Image.fromarray(rgb).save(out/f'actual_theia_rgb_{episode}_{frame}.png')
            if not np.isfinite(case.data.qpos).all() or row['root_up']<.25:
                active_wall += time.perf_counter()-t
                report['status']='stopped_unstable';break
            if viewer and time.perf_counter()-last_display>=1/15:
                viewer.sync();panel.show(rgb,frame,a.rgb_mode);last_display=time.perf_counter()
            if a.realtime:deadline.wait()
            active_wall += time.perf_counter()-t
        else:report['status']='step_limit_not_quality_certified'
        if a.hold_final and viewer:
            while viewer.is_running():viewer.sync();panel.pump();time.sleep(.05)
    except Exception as exc:
        report['error']=str(exc)
        raise
    finally:
        report['steps']=len(rows);report['wall_seconds']=time.perf_counter()-started
        report['sim_seconds_stepped']=len(rows)*.02
        report['active_control_wall_seconds']=active_wall
        report['active_realtime_factor']=len(rows)*.02/active_wall if active_wall else None
        report['realtime_factor_including_setup_and_pause']=len(rows)*.02/report['wall_seconds']
        (out/'validation.json').write_text(json.dumps(report,indent=2))
        (out/'trajectory.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in rows))
        for engine in engines:engine.close()
        if renderer:renderer.close()
        if viewer:viewer.close()
        if panel:panel.close()
        print(json.dumps(report,indent=2))

if __name__=='__main__':main()
