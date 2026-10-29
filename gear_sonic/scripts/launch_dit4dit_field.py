"""Five-pane model-only launcher. No VR. Manual arm then model; O remains bottom stop."""
import argparse
import json
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
commands=[
 ('bottom','cd '+shlex.quote(str(root/'gear_sonic_deploy'))+' && '+shlex.join(['./deploy.sh','--cp','policy/sonic_v1_1/model','--obs-config','policy/sonic_v1_1/observation_config.yaml','--input-type','zmq_manager','--motor-kp-scale','4,10=1.5','--motor-kd-scale','4,10=1.5','--enable-dex1-heartbeat',a.interface])),
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
