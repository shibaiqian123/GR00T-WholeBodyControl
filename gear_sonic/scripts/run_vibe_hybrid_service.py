"""Visual inference worker owned by the hybrid control terminal; loopback only."""
import argparse, json, sys, time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from gear_sonic.utils.vibe_config import load,digest
from gear_sonic.utils.vibe_hybrid import LiveWindow,VisualModels

def main():
    p=argparse.ArgumentParser(__doc__)
    p.add_argument("--config",required=True);p.add_argument("--out",required=True)
    p.add_argument("--library",required=True);p.add_argument("--cache",required=True)
    p.add_argument("--input",choices=["recorded","pico"],default="recorded")
    p.add_argument("--rpc-port",type=int,default=5581);p.add_argument("--frame-port",type=int,default=5580)
    p.add_argument("--manager-port",type=int,default=5556)
    p.add_argument("--ready-port",type=int,default=5583)
    p.add_argument("--switch-limit",type=float,default=.15)
    p.add_argument("--rgb-mode",choices=["normal","gray"],default="normal")
    p.add_argument("--check-only",action="store_true")
    p.add_argument("--rgb-viewer",action="store_true")
    a=p.parse_args()
    if not 0<a.switch_limit<=.3:raise ValueError("Invalid transition guard")
    from gear_sonic.utils.vibe_hybrid import validate_hardware_order
    cfg=load(a.config);validate_hardware_order(cfg);
    from gear_sonic.utils.standalone.native import NativeCase,History
    out=Path(a.out);out.mkdir(parents=True,exist_ok=False)
    models=VisualModels(cfg,a.library,a.cache,out)
    case=NativeCase(cfg["case"],models.agent.manifest)
    panel=None
    report={"status":"blocked","transport":"loopback","input":a.input,"parity":[],"jump_guard_enabled":False}
    try:
        obs=case.observations()
        errors={k:float(np.abs(v-case.initial["oracle__"+k]).max()) for k,v in obs.items()}
        report["observation_errors"]=errors
        if max(errors.values())>1e-5:raise ValueError("Observation parity failed")
        for mode in ("normal","gray"):
            rgb=case.initial["rgb"][0].copy()
            if mode=="gray":rgb[:]=127
            l0,y0=models.infer(obs,rgb,True);l1,y1=models.infer(obs,rgb)
            from PIL import Image
            Image.fromarray(rgb).save(out/("parity_"+mode+".png"))
            row={"mode":mode,"latent_exact":bool(np.array_equal(l0,l1)),
                 "action_max_abs":float(np.abs(y0-y1).max())}
            report["parity"].append(row)
            if not row["latent_exact"] or row["action_max_abs"]>1e-5:raise ValueError("Model parity failed")
        report["status"]="same_state_passed"
        (out/"validation.json").write_text(json.dumps(report,indent=2))
        if a.check_only:return
        if a.rgb_viewer:
            from gear_sonic.utils.standalone.native import RGBPanel
            panel=RGBPanel()
            panel.root.title("Actual Theia RGB - hybrid POSE (waiting for takeover)")
        import zmq
        ctx=zmq.Context()
        rep=ctx.socket(zmq.REP);rep.setsockopt(zmq.LINGER,0);rep.bind(f"tcp://127.0.0.1:{a.rpc_port}")
        frames=ctx.socket(zmq.SUB);frames.setsockopt(zmq.SUBSCRIBE,b"frame");frames.setsockopt(zmq.RCVHWM,8)
        frames.connect(f"tcp://127.0.0.1:{a.frame_port}")
        human=ctx.socket(zmq.SUB);human.setsockopt(zmq.SUBSCRIBE,b"vibe_pose ")
        human.setsockopt(zmq.SUBSCRIBE,b"vibe_prepare ")
        human.connect(f"tcp://127.0.0.1:{a.manager_port}")
        readiness=ctx.socket(zmq.PUB);readiness.setsockopt(zmq.LINGER,0)
        readiness.bind(f"tcp://127.0.0.1:{a.ready_port}")
        poll=zmq.Poller();poll.register(rep,zmq.POLLIN);poll.register(frames,zmq.POLLIN);poll.register(human,zmq.POLLIN)
        live=LiveWindow();snapshot=None;last_epoch=None;last_tick=None;last_mode=None;input_error=None;pose_start_tick=None;visual_count=0
        journal=(out/"control.jsonl").open("w",buffering=1)
        preparation=None;preparation_at=0.;last_probe=0.
        preparation_log=(out/"preparation.jsonl").open("w",buffering=1)
        def receive_human():
            nonlocal preparation,preparation_at,input_error
            packet=human.recv()
            try:
                topic,payload=packet.split(b" ",1)
                value=json.loads(payload)
                if topic==b"vibe_prepare":
                    if not value.get("prepare_id"):raise ValueError("Missing preparation id")
                    preparation=value;preparation_at=time.monotonic()
                else:
                    live.append(value);input_error=None
            except (ValueError,KeyError,TypeError) as exc:input_error=str(exc)
        def pico_observations(request):
            if input_error:raise ValueError(input_error)
            token,query,reference_ns=live.observations(
                case.data.xquat[case.root].astype(np.float32),
                case.data.xpos[case.points].astype(np.float32),
                case.meta["point_body_names"],time.monotonic(),request["heading"])
            obs=case.observations()
            obs["tokenizer"]=token;obs["augmentation"]=query
            return obs,reference_ns
        def probe(request,meta,rgb):
            nonlocal last_probe
            now=time.monotonic()
            if preparation is None or now-preparation_at>.5 or now-last_probe<.05:return
            last_probe=now
            status=dict(preparation,ready=False,samples=len(live.rows),
                        epoch=meta["epoch"],tick=meta["tick"])
            try:
                if a.input!="pico":raise ValueError("Readiness handshake requires PICO input")
                if not preparation.get("session") or preparation["session"]!=live.session:
                    raise ValueError("Waiting for matching PICO session")
                obs,_=pico_observations(request)
                # The real mode switch rebuilds history from the current state.
                obs["policy"]=History(case.terms()).flat()
                pixels=np.full_like(rgb,127) if a.rgb_mode=="gray" else rgb
                _,action=models.infer(obs,pixels)
                if cfg["clip_actions"] is not None:action=np.clip(action,-cfg["clip_actions"],cfg["clip_actions"])
                target=case.default+case.scale*action.reshape(29)
                if not np.isfinite(target).all():raise ValueError("Nonfinite preflight target")
                jump=float(np.abs(target-case.data.ctrl[case.aids]).max())
                status["switch_jump"]=jump
                status["jump_guard_enabled"]=False
                status["jump_exceeds_diagnostic_limit"]=jump>a.switch_limit
                status.update(ready=True,reason="PICO window and visual transition ready")
            except Exception as exc:status["reason"]=str(exc)
            status["sent_monotonic"]=time.monotonic()
            readiness.send(b"vibe_ready "+json.dumps(status).encode())
            preparation_log.write(json.dumps(status)+"\n")
        def receive_frame():
            nonlocal snapshot,last_epoch,last_tick,last_mode,live,pose_start_tick,input_error,preparation
            parts=frames.recv_multipart()
            if len(parts)!=3:raise ValueError("Bad simulator packet")
            meta=json.loads(parts[1]);rgb=np.frombuffer(parts[2],np.uint8).reshape(63,112,3).copy()
            if meta["bundle_sha256"]!=cfg["hashes"]["bundle"]:raise ValueError("Simulator model mismatch")
            epoch,tick=int(meta["epoch"]),int(meta["tick"])
            if epoch!=last_epoch:
                live=LiveWindow();last_mode=None;last_tick=None;last_epoch=epoch;pose_start_tick=None;input_error=None;preparation=None
            if last_tick is not None and tick!=last_tick and tick!=last_tick+20:
                raise ValueError("Simulator skipped control frames")
            for key in ("qpos","qvel","ctrl"):
                value=np.asarray(meta[key],float)
                if value.shape!=getattr(case.data,key).shape or not np.isfinite(value).all():
                    raise ValueError("Invalid simulator "+key)
                getattr(case.data,key)[:]=value
            case.mujoco.mj_forward(case.model,case.data)
            case.last=((case.data.ctrl[case.aids]-case.default)/case.scale).astype(np.float32)
            if last_tick is None:case.history=History(case.terms())
            elif tick!=last_tick:case.history.append(case.terms())
            last_tick=tick
            snapshot=(meta,rgb,time.monotonic())
        print("HYBRID_VISUAL_READY: same-state parity passed; awaiting simulator/controller",flush=True)
        while True:
            events=dict(poll.poll(20 if panel else 100))
            if panel:panel.pump()
            if human in events:
                while human.poll(0):receive_human()
            if frames in events:receive_frame()
            if rep in events:
                request=rep.recv_json()
                try:
                    deadline=time.monotonic()+.15
                    while (snapshot is None or snapshot[0]["tick"]!=int(request["tick"])) and time.monotonic()<deadline:
                        if frames.poll(10):receive_frame()
                    if snapshot is None:raise ValueError("Waiting for simulation RGB")
                    meta,rgb,received=snapshot
                    if time.monotonic()-received>.5:raise ValueError("Simulation stream stale")
                    if int(request["tick"])!=meta["tick"] or int(request["epoch"])!=meta["epoch"]:
                        raise ValueError("State/image clock mismatch")
                    mode=request["mode"]
                    if mode not in ("POSE","SONIC"):raise ValueError("Unsupported mode")
                    # Restore history in target convention using actually applied joint targets.
                    if mode!=last_mode:case.history=History(case.terms())
                    if mode=="POSE":
                        if a.input=="pico":
                            while human.poll(0):receive_human()
                            obs,reference_ns=pico_observations(request)
                        else:
                            if last_mode!="POSE":pose_start_tick=meta["tick"]
                            case.frame=case.meta["initial_frame"]+(meta["tick"]-pose_start_tick)//20
                            if case.frame>=case.meta["frames"]:raise ValueError("Recorded reference ended")
                            obs=case.observations();reference_ns=case.frame*20_000_000
                        if a.rgb_mode=="gray":rgb=np.full_like(rgb,127)
                        _,action=models.infer(obs,rgb)
                        visual_count+=1
                        if panel:panel.show(rgb,meta["tick"]//20,a.rgb_mode)
                        if visual_count==1 or visual_count%50==0:
                            from PIL import Image
                            name=f"theia_epoch{meta['epoch']}_tick{meta['tick']}_{a.rgb_mode}.png"
                            Image.fromarray(rgb).save(out/name)
                            (out/(name+".json")).write_text(json.dumps({
                                "epoch":meta["epoch"],"tick":meta["tick"],"mode":a.rgb_mode,
                                "source":"actual RGB consumed by Theia","reference_ns":reference_ns,
                                "band_enabled":meta.get("band_enabled"),"bundle_sha256":cfg["hashes"]["bundle"]},indent=2))
                        if cfg["clip_actions"] is not None:action=np.clip(action,-cfg["clip_actions"],cfg["clip_actions"])
                        target=case.default+case.scale*action.reshape(29)
                    else:
                        target=np.asarray(request["target"],np.float32);reference_ns=None
                        while human.poll(0):receive_human()
                        probe(request,meta,rgb)
                    if target.shape!=(29,) or not np.isfinite(target).all():raise ValueError("Invalid target")
                    jump=float(np.abs(target-case.data.ctrl[case.aids]).max())
                    # Simulation-only: user disabled jump rejection; retain diagnostics.
                    if last_mode is not None and mode!=last_mode:
                        print(f"HYBRID_SWITCH: jump={jump:.6f} rad; jump guard disabled (simulation only)",flush=True)
                    reply={"ok":True,"tick":meta["tick"],"epoch":meta["epoch"],"target":target.tolist(),
                           "mode":mode,"reference_ns":reference_ns,"switch_jump":jump,"jump_guard_enabled":False}
                    last_mode=mode
                except Exception as exc:
                    reply={"ok":False,"error":str(exc)}
                rep.send_json(reply);journal.write(json.dumps(reply)+"\n")
    finally:
        (out/"validation.json").write_text(json.dumps(report,indent=2))
        if panel:panel.close()
        models.close()
if __name__=="__main__":main()
