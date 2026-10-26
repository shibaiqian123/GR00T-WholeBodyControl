"""Training-asset MuJoCo DDS simulator for the opt-in hybrid controller. Loopback only."""
import argparse,json,sys,time,threading,secrets
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from gear_sonic.utils.vibe_config import load
MAGIC=0x56494245


def dds_bridge(endpoint):
    """Run under the already verified teleop Python/SDK, owned by Terminal 1."""
    import os,zmq
    sys.path.insert(0,str(ROOT/"external_dependencies/unitree_sdk2_python"))
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize,ChannelPublisher,ChannelSubscriber
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_,unitree_hg_msg_dds__IMUState_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_,LowCmd_,IMUState_
    ChannelFactoryInitialize(0,"lo")
    state=unitree_hg_msg_dds__LowState_();imu=unitree_hg_msg_dds__IMUState_()
    pub=ChannelPublisher("rt/lowstate",LowState_);pub.Init()
    torso=ChannelPublisher("rt/secondary_imu",IMUState_);torso.Init()
    latest=[None];lock=threading.Lock()
    def receive(msg):
        if msg.reserve[0]!=MAGIC:return
        with lock:
            latest[0]={"reserve":list(msg.reserve),"target":[msg.motor_cmd[i].q for i in range(29)]}
    sub=ChannelSubscriber("rt/lowcmd",LowCmd_);sub.Init(receive,8)
    ctx=zmq.Context();rep=ctx.socket(zmq.REP)
    rep.setsockopt(zmq.LINGER,0);rep.setsockopt(zmq.RCVTIMEO,1000);rep.bind(endpoint)
    parent=os.getppid()
    try:
        while os.getppid()==parent:
            try:r=rep.recv_json()
            except zmq.Again:continue
            state.tick=r["tick"];state.reserve[0]=r["epoch"];state.reserve[1]=MAGIC
            for i in range(29):
                state.motor_state[i].q=r["q"][i];state.motor_state[i].dq=r["dq"][i]
                state.motor_state[i].tau_est=r["tau"][i]
            state.imu_state.quaternion[:]=r["quat"];state.imu_state.gyroscope[:]=r["gyro"]
            imu.quaternion[:]=r["torso_quat"];imu.gyroscope[:]=r["torso_gyro"]
            pub.Write(state);torso.Write(imu)
            with lock:reply=latest[0]
            rep.send_json(reply)
    finally:
        rep.close();ctx.term()


def main():
    p=argparse.ArgumentParser(__doc__)
    p.add_argument("--config",required=True);p.add_argument("--out",required=True)
    p.add_argument("--viewer",action="store_true");p.add_argument("--steps",type=int,default=10000)
    p.add_argument("--frame-port",type=int,default=5580)
    p.add_argument("--initialization",choices=["standing","seed"],default="standing")
    p.add_argument("--check-only",action="store_true")

    a=p.parse_args()
    from gear_sonic.utils.vibe_hybrid import validate_hardware_order
    cfg=load(a.config);validate_hardware_order(cfg);
    import mujoco,zmq
    from gear_sonic.utils.standalone.native import NativeCase,Deadline
    manifest=json.loads((Path(cfg["bundle"]).parent/"visual_decoder.manifest.json").read_text())
    case=NativeCase(cfg["case"],manifest)
    from gear_sonic.utils.vibe_sim_support import standing_reset,original_band,apply_band
    band,band_sha=original_band()
    band.enable=a.initialization=="standing"
    def initialize():
        if a.initialization=="standing":
            result=standing_reset(case)
            band.enable=True;band.length=0;band.reference_heading=result["heading_radians"]
            band.point=np.array([*result["root_position"][:2],result["floor_height"]+1.])
        else:
            case.reset();band.enable=False
            result={"mode":"seed","frame":case.frame}
        return result
    initialization=initialize()
    import subprocess,signal
    out=Path(a.out);out.mkdir(parents=True,exist_ok=False)
    (out/"initialization.json").write_text(json.dumps(dict(initialization,band_source_sha256=band_sha,
        band_enabled=band.enable,camera=cfg["camera"],case=cfg["case"]),indent=2))
    if a.check_only:
        print("HYBRID_INIT_CHECK_PASSED",json.dumps(initialization),flush=True)
        return
    signal.signal(signal.SIGTERM,lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    ctx=zmq.Context();images=ctx.socket(zmq.PUB);images.setsockopt(zmq.LINGER,0)
    images.bind(f"tcp://127.0.0.1:{a.frame_port}")
    endpoint="ipc://"+str(out/"dds.sock")
    bridge=subprocess.Popen([str(ROOT/".venv_teleop/bin/python"),str(Path(__file__).resolve()),"--dds-bridge",endpoint],
                            env={k:v for k,v in __import__("os").environ.items() if k not in ("PYTHONPATH","LD_LIBRARY_PATH")})
    transport=ctx.socket(zmq.REQ);transport.setsockopt(zmq.LINGER,0)
    transport.setsockopt(zmq.RCVTIMEO,3000);transport.setsockopt(zmq.SNDTIMEO,3000);transport.connect(endpoint)
    camera=cfg["camera"];renderer=mujoco.Renderer(case.model,height=63,width=112)
    option=mujoco.MjvOption();option.geomgroup[:]=0;option.geomgroup[camera["geom_groups"]]=1
    option.flags[mujoco.mjtVisFlag.mjVIS_TEXTURE]=camera["textures"]
    viewer_threads=[];viewer=None;keys=[];epoch=secrets.randbelow(2**31-1)+1;tick=0;paused=False;count=0;fault_reason=None
    report={"status":"waiting_controller","steps":0,"simulation_only":True,"epoch":epoch}
    log=(out/"steps.jsonl").open("w",buffering=1)
    if a.viewer:
        import mujoco.viewer
        existing_threads=set(threading.enumerate())
        viewer=mujoco.viewer.launch_passive(case.model,case.data,key_callback=keys.append)
        viewer_threads=[t for t in threading.enumerate() if t not in existing_threads]
        viewer.cam.lookat[:]=case.data.xpos[case.root];viewer.cam.distance=3.
    clock=Deadline(.02);started=time.monotonic();last_view=0;last_cmd=None;active_started=None
    last_ui=None
    def show_status():
        nonlocal last_ui
        status=("FAULT - press R" if fault_reason else "PAUSED" if paused else
                "WAITING FOR TERMINAL 2 - physics held" if last_cmd is None else "RUNNING")
        text=f"{status} | band={'ON' if band.enable else 'OFF'} length={band.length:.2f} | step={count}"
        if viewer:
            viewer.set_texts((None,None,"Native MuJoCo (standalone)\n"+text,
                "7/8 band length | 9 band ON/OFF\nSPACE pause | R reset\n"+(fault_reason or "")))
        if text!=last_ui and (last_ui is None or not status=="RUNNING"):
            print("SIM_UI:",text,flush=True)
            (out/"ui_status.json").write_text(json.dumps(dict(status=status,band=bool(band.enable),
                band_length=float(band.length),step=count,fault=fault_reason),indent=2))
        last_ui=text

    print("HYBRID_SIM_READY: training fixed Dex1, loopback DDS, 112x63 RGB, no inference; band="+str(band.enable),flush=True)
    print("7=lower band, 8=raise band, 9=toggle band, SPACE=pause, R=reset (restart controller)",flush=True)
    try:
        while count<a.steps and (viewer is None or viewer.is_running()):
            try:
                while keys:
                    key=keys.pop(0)
                    print("SIM_KEY:",key,flush=True)
                    # Accept both the top number row and keypad digits.
                    key={327:55,328:56,329:57}.get(key,key)
                    if key==32:
                        if fault_reason:print("FAULT remains paused; press R to reset first",flush=True)
                        else:paused=not paused;clock.resume()
                    if key in (82,114):
                        initialization=initialize();epoch+=1;tick=0;count=0;last_cmd=None;active_started=None;fault_reason=None
                        transport.close(0)
                        transport=ctx.socket(zmq.REQ);transport.setsockopt(zmq.LINGER,0)
                        transport.setsockopt(zmq.RCVTIMEO,3000);transport.setsockopt(zmq.SNDTIMEO,3000)
                        transport.connect(endpoint)
                        # A new controller process is required to clear SONIC/planner state.
                        paused=True
                        print("RESET_PAUSED: restart Terminal 2, then SPACE; no automatic takeover",flush=True)
                        print("RESET: restart hybrid controller; stale commands will be refused",flush=True)
                        clock.resume()
                    if key in (55,56,57):
                        band.MujuocoKeyCallback(key)
                        print("BAND:",band.enable,"length:",band.length,flush=True)
                show_status()
                if fault_reason:
                    if viewer:viewer.sync()
                    time.sleep(.02)
                    continue
                mujoco.mj_forward(case.model,case.data)
                renderer.update_scene(case.data,camera=case.cam,scene_option=option)
                renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=camera["shadows"]
                rgb=renderer.render()
                if count==0 and not (out/"camera_initial.png").exists():
                    from PIL import Image
                    Image.fromarray(rgb).save(out/"camera_initial.png")
                rot=case.data.xmat[case.root].reshape(3,3)
                body={"tick":tick,"epoch":epoch,"q":case.data.qpos[case.qids].tolist(),
                      "dq":case.data.qvel[case.vids].tolist(),"tau":case.data.qfrc_actuator[case.vids].tolist(),
                      "quat":case.data.xquat[case.root].tolist(),"gyro":(rot.T@case.data.cvel[case.root,:3]).tolist(),
                      "torso_quat":case.data.xquat[case.torso].tolist(),
                      "torso_gyro":(case.data.xmat[case.torso].reshape(3,3).T@case.data.cvel[case.torso,:3]).tolist()}
                meta={"epoch":epoch,"tick":tick,"sim_time":float(case.data.time),
                      "bundle_sha256":cfg["hashes"]["bundle"],"band_enabled":band.enable,"initialization":a.initialization,
                      **{k:getattr(case.data,k).tolist() for k in ("qpos","qvel","ctrl")}}
                deadline=time.monotonic()+1.
                waiting_started=time.monotonic()
                command=None
                while command is None:
                    transport.send_json(body)
                    msg=transport.recv_json()
                    images.send_multipart([b"frame",json.dumps(meta).encode(),rgb.tobytes()])
                    now=time.monotonic()
                    if viewer and now-last_view>=1/30:show_status();viewer.sync();last_view=now
                    if viewer and (not viewer.is_running() or keys):break
                    if msg is not None and msg["reserve"][2]==epoch:
                        if msg["reserve"][3]:
                            if last_cmd is None:
                                time.sleep(.01);continue
                            raise RuntimeError("Controller fault; simulation paused")
                        if msg["reserve"][1]==tick:command=msg
                    if command is None:
                        if last_cmd is not None and now>deadline:
                            raise RuntimeError("Controller did not acknowledge control step; simulation paused")
                        time.sleep(.01)
                if command is None:continue
                if paused:
                    time.sleep(.02);continue
                if last_cmd is None:
                    clock.resume();active_started=time.monotonic()
                targets=np.asarray(command["target"])
                if not np.isfinite(targets).all():raise ValueError("Nonfinite DDS target")
                # Compiled training actuators own PD and effort limits; DDS gains are intentionally not applied twice.
                case.data.ctrl[case.aids]=targets
                for _ in range(4):
                    apply_band(case,band)
                    mujoco.mj_step(case.model,case.data)
                tick+=20;count+=1;last_cmd=time.monotonic()
                if not np.isfinite(case.data.qpos).all():raise RuntimeError("Nonfinite physics")
                mujoco.mj_forward(case.model,case.data)
                root_up=float(case.data.xmat[case.root].reshape(3,3)[2,2])
                contact_min=min([float(case.data.contact[i].dist) for i in range(case.data.ncon)] or [0.])
                contact_pair=None
                if case.data.ncon:
                    worst=min(range(case.data.ncon),key=lambda i:case.data.contact[i].dist)
                    contact=case.data.contact[worst]
                    contact_pair=[mujoco.mj_id2name(case.model,mujoco.mjtObj.mjOBJ_GEOM,int(g)) or str(int(g))
                                  for g in (contact.geom1,contact.geom2)]
                log.write(json.dumps({"tick":tick,"sim_time":float(case.data.time),"wall":time.monotonic()-started,
                                      "root_up":root_up,"min_contact_distance":contact_min,"contact_pair":contact_pair,
                                      "band_enabled":band.enable,"band_wrench":case.data.xfrc_applied[case.root].tolist(),
                                      "joint_target":targets.tolist(),"joint_actual":case.data.qpos[case.qids].tolist(),
                                      "controller_wait":last_cmd-waiting_started,
                                      "max_abs_torque":float(np.abs(case.data.qfrc_actuator).max())})+"\n")
                if root_up < .25:raise RuntimeError("Severe loss of upright pose; simulation paused")
                clock.wait()
            except Exception as exc:
                fault_reason=str(exc);paused=True;clock.resume()
                report["status"]="fault_paused";report["reason"]=fault_reason
                (out/"fault.json").write_text(json.dumps(dict(reason=fault_reason,epoch=epoch,tick=tick),indent=2))
                print("HYBRID_SIM_FAULT:",exc,"| R resets, SPACE cannot bypass fault",flush=True)
                if not viewer:raise
        report["status"]="ended"
    finally:
        report["steps"]=count;report["sim_time"]=float(case.data.time)
        report["wall_seconds"]=time.monotonic()-started
        report["rtf_including_startup"]=float(case.data.time)/max(report["wall_seconds"],1e-9)
        report["active_wall_seconds"]=None if active_started is None else time.monotonic()-active_started
        report["rtf_after_controller_ready"]=None if active_started is None else float(case.data.time)/max(report["active_wall_seconds"],1e-9)
        report["band_enabled_at_end"]=band.enable
        report["initialization"]=initialization
        (out/"report.json").write_text(json.dumps(report,indent=2))
        log.close()
        if viewer:viewer.close()
        for thread in viewer_threads:thread.join(timeout=5)
        renderer.close()
        images.close();transport.close();ctx.term()
        if bridge.poll() is None:
            bridge.terminate()
            try:bridge.wait(timeout=5)
            except subprocess.TimeoutExpired:bridge.kill();bridge.wait()
if __name__=="__main__":
    if len(sys.argv)==3 and sys.argv[1]=="--dds-bridge":dds_bridge(sys.argv[2])
    else:main()
