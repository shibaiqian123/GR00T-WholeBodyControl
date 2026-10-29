"""Five-pane model-only launcher. No VR. Manual arm then model; O remains bottom stop."""
import argparse
import json
import hashlib
import math
import os
from pathlib import Path
import shlex
import socket
import subprocess
import time

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--session',default='dit4dit_field')
p.add_argument('--policy-ssh',default='2025slx@10.246.1.32',help='SSH inference host; service must already be running on its loopback 5560')
p.add_argument('--interface',default='enx6c1ff7b8d821')
p.add_argument('--task-prompt',default='Pick up the cup on the left with your left gripper. Then pick up the red block on the right with your right gripper and place it into the cup held in your left gripper. Finally, place the cup, with the red block inside, back on the table using your left gripper.')
p.add_argument('--inference-mode',choices=('async','sync'),default='async')
p.add_argument('--segment-frames',type=int,default=10)
p.add_argument('--joint-target-limits',type=Path,default=os.environ.get('SONIC_JOINT_TARGET_LIMITS'),help='Explicit continuous 29-joint limiter JSON; defaults to caller SONIC_JOINT_TARGET_LIMITS, otherwise disabled')
p.add_argument('--arm-target-smoothing',type=Path,default=os.environ.get('SONIC_ARM_TARGET_SMOOTHING'),help='Experimental shoulder/elbow blend + lowpass + slew-limit JSON, disabled by default')
p.add_argument('--dry-run',action='store_true')
p.add_argument('--no-attach',action='store_true')
a=p.parse_args()
if not 1<=a.segment_frames<=50:p.error("--segment-frames must be 1..50")
if a.inference_mode=="sync" and a.segment_frames==50:p.error("50 frames exceed the unchanged action horizon once inference takes time")
if not a.session or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in a.session):p.error('Invalid session name')
root=Path(__file__).resolve().parents[2];session=a.session
out=root/'outputs'/('model_only_'+time.strftime('%Y%m%d_%H%M%S'))
limits='gear_sonic/configs/dex1_limits.g1_20261013.json'
tele=str(root/'.venv_teleop/bin/python');data=str(root/'.venv_data_collection/bin/python');task=a.task_prompt
# Resolve in the calling process: tmux server environment may be stale.
limiter_config=None
limiter_source=None
if a.joint_target_limits is not None:
    try:
        limiter_source=a.joint_target_limits.expanduser().resolve(strict=True)
        limiter_config=json.loads(limiter_source.read_text())
        if limiter_config.get('units')!='rad' or limiter_config.get('joint_order')!='hardware_29':
            raise ValueError('units=rad and joint_order=hardware_29 required')
        if not isinstance(limiter_config.get('constraint_source'),str) or not limiter_config['constraint_source'].strip():
            raise ValueError('constraint_source required')
        if limiter_config.get('simulation_only',False) is not False:
            raise ValueError('simulation-only limits cannot be used by this real-robot launcher')
        for key in ('q_min','q_max','max_velocity','max_tracking_error'):
            values=limiter_config.get(key)
            if not isinstance(values,list) or len(values)!=29 or any(type(v) not in (int,float) or not math.isfinite(v) for v in values):
                raise ValueError(key+' must contain 29 finite numbers')
        for i in range(29):
            if limiter_config['q_min'][i]>=limiter_config['q_max'][i] or limiter_config['max_velocity'][i]<=0 or limiter_config['max_tracking_error'][i]<=0:
                raise ValueError('invalid bounds at hardware joint '+str(i))
        if os.environ.get('SONIC_SINGLE_STEP_CONFIG') or os.environ.get('VIBE_HYBRID_ENDPOINT'):
            raise ValueError('continuous limiter incompatible with single-step/hybrid')
    except (OSError,ValueError,TypeError,AttributeError) as e:
        p.error('joint target limiter: '+str(e))
arm_config=None
arm_source=None
if a.arm_target_smoothing is not None:
    try:
        if limiter_config is not None:raise ValueError('choose arm smoothing OR full-body target limiter, not both')
        arm_source=a.arm_target_smoothing.expanduser().resolve(strict=True)
        arm_config=json.loads(arm_source.read_text())
        if arm_config.get('simulation_only') is not False:raise ValueError('real-robot launcher requires simulation_only=false')
        if arm_config.get('units')!='rad' or arm_config.get('joint_order')!='hardware_29':raise ValueError('rad hardware_29 required')
        for key in ('T_blend','tau','v_max'):
            value=arm_config.get(key)
            if type(value) not in (int,float) or not math.isfinite(value) or value<=0:raise ValueError(key+' must be finite and positive')
        for key in ('q_min','q_max'):
            values=arm_config.get(key)
            if not isinstance(values,list) or len(values)!=29 or any(type(v) not in (int,float) or not math.isfinite(v) for v in values):raise ValueError(key+' needs 29 finite values')
        indices=(15,16,17,18,22,23,24,25)
        expected=[side+'_'+joint+'_joint' for side in ('left','right') for joint in ('shoulder_pitch','shoulder_roll','shoulder_yaw','elbow')]
        names=arm_config.get('joint_names',[])
        if len(names)!=29 or [names[i] for i in indices]!=expected:raise ValueError('shoulder/elbow mapping mismatch')
        if any(arm_config['q_min'][i]>=arm_config['q_max'][i] for i in indices):raise ValueError('invalid arm position bounds')
        if os.environ.get('SONIC_SINGLE_STEP_CONFIG') or os.environ.get('VIBE_HYBRID_ENDPOINT'):raise ValueError('arm smoothing incompatible with single-step/hybrid')
    except (OSError,ValueError,TypeError,AttributeError) as e:
        p.error('arm target smoothing: '+str(e))
# Use a validated per-run copy; preserve the exact configuration with the logs.
limiter_run_path=out/'joint_target_limits.json'
bottom_env=['env','-u','SONIC_JOINT_TARGET_LIMITS','-u','SONIC_ARM_TARGET_SMOOTHING']
if limiter_config is not None:
    bottom_env += ['SONIC_JOINT_TARGET_LIMITS='+str(limiter_run_path),
                   'SONIC_STARTUP_AUDIT='+str(out/'bottom_target_audit.jsonl')]
    print('JOINT TARGET LIMITER: enabled; config='+str(limiter_source)+'; validation checks format only, not hardware safety')
else:
    print('JOINT TARGET LIMITER: disabled (explicitly clears inherited tmux setting)')
arm_run_path=out/'arm_target_smoothing.json'
if arm_config is not None:
    bottom_env += ['SONIC_ARM_TARGET_SMOOTHING='+str(arm_run_path),
                   'SONIC_STARTUP_AUDIT='+str(out/'bottom_target_audit.jsonl')]
    print('ARM TARGET SMOOTHING: enabled (experimental); T_blend='+str(arm_config['T_blend'])+' tau='+str(arm_config['tau'])+' v_max='+str(arm_config['v_max'])+'; shoulders/elbows only; no token filtering')
else:
    print('ARM TARGET SMOOTHING: disabled')
commands=[
 ('bottom','cd '+shlex.quote(str(root/'gear_sonic_deploy'))+' && '+shlex.join(bottom_env+['./deploy.sh','--cp','policy/sonic_v1_1/model','--obs-config','policy/sonic_v1_1/observation_config.yaml','--input-type','zmq_manager','--motor-kp-scale','4,10=1.5','--motor-kd-scale','4,10=1.5','--enable-dex1-heartbeat',a.interface])),
 ('observations',shlex.join([tele,'-u','-m','gear_sonic.scripts.run_model_observers','--interface',a.interface])),
 ('dex1',shlex.join([tele,'-u','-m','gear_sonic.scripts.run_dex1_trigger_control','--interface',a.interface,'--limits',limits,'--model-only','--target-port','5563','--telemetry-port','5559','--travel-time','2','--obstruction-hold','--obstruction-timeout','0.5','--execute'])),
 ('gateway',shlex.join([data,'-u','-m','gear_sonic.scripts.run_dit4dit_model','--task-prompt',task,'--limits',limits,'--log-dir',str(out/'gateway'),'--inference-mode',a.inference_mode,'--segment-frames',str(a.segment_frames),'--execute'])),
 ('tunnel',shlex.join(['ssh','-N','-T','-o','ExitOnForwardFailure=yes','-o','ServerAliveInterval=10','-o','ServerAliveCountMax=3','-L','127.0.0.1:5560:127.0.0.1:5560',a.policy_ssh]))]
if a.inference_mode=='sync':
    print('SYNC: arm holds grippers; manual model requests a fresh prediction then starts. SONIC continues last token during inference; NOT stationary hold. Existing age/stop limits apply.')
if a.dry_run:
    print('REMOTE MODEL: no local DiT4DiT GPU load. Start policy on server first; tunnel pane may ask for SSH password. Manual arm/model.')
    for name,command in commands:print(name+'\n'+command+'\n')
    raise SystemExit(0)
if subprocess.run(['tmux','has-session','-t',session],capture_output=True).returncode==0:
    raise SystemExit('Session exists; use: tmux attach -t '+session)
for port in (5556,5557,5559,5560,5562,5563,5564):
    with socket.socket() as sock:
        # Match ZMQ reuse semantics: ignore retired connections, never active listeners.
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        try:sock.bind(('127.0.0.1',port))
        except OSError:raise SystemExit(f'Port {port} occupied; inspect existing service first. Nothing started.')
for required in (Path(tele),Path(data),root/limits):
    if not required.exists():raise SystemExit('Missing: '+str(required))
out.mkdir(parents=True,exist_ok=False)
if limiter_config is not None:
    limiter_run_path.write_text(json.dumps(limiter_config,indent=2,allow_nan=False)+'\n')
    (out/'joint_target_limits_source.json').write_text(json.dumps(dict(source=str(limiter_source),saved_sha256=hashlib.sha256(limiter_run_path.read_bytes()).hexdigest()),indent=2))
if arm_config is not None:
    arm_run_path.write_text(json.dumps(arm_config,indent=2,allow_nan=False)+'\n')
    (out/'arm_target_smoothing_source.json').write_text(json.dumps(dict(source=str(arm_source),saved_sha256=hashlib.sha256(arm_run_path.read_bytes()).hexdigest()),indent=2))
def tm(*args):
    return subprocess.check_output(['tmux',*args],text=True).strip()
# Build the complete layout before starting any control process.
first=tm('new-session','-d','-s',session,'-n','field','-x','220','-y','64','-c',str(root),'-P','-F','#{pane_id}')
window=tm('display-message','-p','-t',first,'#{window_id}')
panes=[first]
for _ in range(4):
    panes.append(tm('split-window','-d','-t',panes[-1],'-c',str(root),'-P','-F','#{pane_id}'))
    tm('select-layout','-t',window,'tiled')
tm('set-option','-t',session,'mouse','on')
tm('set-option','-w','-t',window,'synchronize-panes','off')
tm('set-option','-w','-t',window,'pane-active-border-style','fg=green,bold')
tm('set-option','-w','-t',window,'pane-border-status','top')
tm('set-option','-w','-t',window,'pane-border-format',' #{pane_index}: #{pane_title} ')
for pane,(name,command) in zip(panes,commands):
    tm('select-pane','-t',pane,'-T',name)
    tm('pipe-pane','-o','-t',pane,'cat >> '+shlex.quote(str(out/(name+'.log'))))
    tm('send-keys','-t',pane,'-l',command)
    tm('send-keys','-t',pane,'Enter')
tm('select-pane','-t',first)
(out/'session.json').write_text(json.dumps(dict(session=session,panes=dict(zip([x[0] for x in commands],panes)),commands=dict(commands)),indent=2))
print('Logs:',out,flush=True)
print('Click a pane or Ctrl+B then arrows: select input focus; Ctrl+B then z: zoom. Bottom O stops; gateway mode changes are manual.',flush=True)
if not a.no_attach:
    if os.environ.get('TMUX'):subprocess.run(['tmux','switch-client','-t',session],check=True)
    else:subprocess.run(['tmux','attach-session','-t',session],check=True)
