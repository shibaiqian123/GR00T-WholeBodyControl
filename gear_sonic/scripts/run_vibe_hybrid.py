"""Opt-in launcher: original C++ manager/state machine plus local visual worker."""
import argparse,json,os,shlex,subprocess,sys,time,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from gear_sonic.utils.vibe_config import load

def build(base,out):
    out.mkdir(parents=True,exist_ok=True)
    entries=json.loads((base/"build/compile_commands.json").read_text())
    e=next(e for e in entries if e["file"].endswith("/src/g1_deploy_onnx_ref.cpp"))
    original=base/"target/release/g1_deploy_onnx_ref"
    before=hashlib.sha256(original.read_bytes()).hexdigest()
    command=[x for x in shlex.split(e["command"]) if x!="-ffast-math"]
    command[command.index("-o")+1]=str(out/"hybrid.o")
    with (out/"build.log").open("w") as log:
        subprocess.run(command,cwd=e["directory"],stdout=log,stderr=subprocess.STDOUT,check=True)
        link=shlex.split((Path(e["directory"])/"CMakeFiles/g1_deploy_onnx_ref.dir/link.txt").read_text())
        link=[str(out/"hybrid.o") if x.endswith("g1_deploy_onnx_ref.cpp.o") else x for x in link]
        link[link.index("-o")+1]=str(out/"g1_deploy_hybrid")
        subprocess.run(link,cwd=e["directory"],stdout=log,stderr=subprocess.STDOUT,check=True)
    if hashlib.sha256(original.read_bytes()).hexdigest()!=before:raise RuntimeError("Original binary changed")
    return out/"g1_deploy_hybrid"

def main():
    p=argparse.ArgumentParser(__doc__)
    p.add_argument("--policy-type",choices=["vibe-hybrid"],required=True)
    p.add_argument("--cp",required=True);p.add_argument("--obs-config",required=True)
    p.add_argument("--vibe-obs-config",required=True);p.add_argument("--bundle",required=True)
    p.add_argument("--vision-encoder",required=True)
    p.add_argument("--input-type",choices=["zmq_manager"],default="zmq_manager")
    p.add_argument("--visual-input",choices=["recorded","pico"],default="pico")
    p.add_argument("--out",required=True);p.add_argument("--check-only",action="store_true")
    p.add_argument("--build-only",action="store_true")
    p.add_argument("--trt-library",default="/home/cqu023/sonic-diagnostics/vibe-deploy-build-v1/libvibe_trt.so")
    p.add_argument("--engine-cache",default="/home/cqu023/sonic-diagnostics/groot-vibe-v1/check/engine_cache")
    p.add_argument("--motion-data",default="reference/example")
    p.add_argument("--planner",default="planner/target_vel/V2/planner_sonic.onnx")
    p.add_argument("--rgb-mode",choices=["normal","gray"],default="normal")
    p.add_argument("--rgb-viewer",action="store_true")
    p.add_argument("target",choices=["sim"])
    a=p.parse_args()
    base=ROOT/"gear_sonic_deploy"
    cfg=load(a.vibe_obs_config)
    b=Path(a.bundle).resolve()
    if b.is_dir():b/="bundle.json"
    if str(b)!=cfg["bundle"] or str(Path(a.vision_encoder).resolve())!=cfg["vision_encoder"]:
        p.error("Model CLI paths disagree with ViBe contract")
    for path in (Path(str(a.cp)+"_encoder.onnx"),Path(str(a.cp)+"_decoder.onnx"),
                 Path(a.obs_config),Path(a.planner)):
        if not (base/path).is_file():p.error("Missing SONIC asset: "+str(path))
    out=Path(a.out).resolve();out.mkdir(parents=True,exist_ok=False)
    env=os.environ.copy()
    env["PYTHONPATH"]=str(ROOT)
    env["PYTHONNOUSERSITE"]="1"
    env["LD_LIBRARY_PATH"]="/opt/onnxruntime/lib:/usr/local/cuda-12.1/lib64:"+env.get("LD_LIBRARY_PATH","")
    worker=[sys.executable,str(ROOT/"gear_sonic/scripts/run_vibe_hybrid_service.py"),
        "--config",str(Path(a.vibe_obs_config).resolve()),"--out",str(out/"visual"),
        "--library",a.trt_library,"--cache",a.engine_cache,"--input",a.visual_input,"--rgb-mode",a.rgb_mode]
    if a.rgb_viewer:worker.append("--rgb-viewer")
    if a.check_only:
        subprocess.run(worker+["--check-only"],env=env,check=True);return
    binary=build(base,out/"build")
    if a.build_only:
        print("HYBRID_BUILD_ONLY",binary);return
    print("SIMULATION ONLY | PLANNER/VR3PT=SONIC | POSE=ViBe | input="+a.visual_input,flush=True)
    with (out/"visual.log").open("w") as log:
        process=subprocess.Popen(worker,env=env,stdout=log,stderr=subprocess.STDOUT)
        control=None
        try:
            until=time.monotonic()+180
            while "HYBRID_VISUAL_READY" not in (out/"visual.log").read_text():
                if process.poll() is not None:raise RuntimeError("Visual worker failed; see visual.log")
                if time.monotonic()>until:raise RuntimeError("Visual initialization timeout")
                time.sleep(.2)
            command=[str(binary),"lo",str(a.cp)+"_decoder.onnx",a.motion_data,
                "--encoder-file",str(a.cp)+"_encoder.onnx","--obs-config",a.obs_config,
                "--planner-file",a.planner,"--input-type","zmq_manager",
                "--zmq-host","127.0.0.1","--zmq-port","5556","--disable-crc-check",
                "--policy-precision","32","--planner-precision","32",
                "--vibe-hybrid-endpoint","tcp://127.0.0.1:5581"]
            (out/"command.json").write_text(json.dumps(command,indent=2))
            control=subprocess.Popen(command,cwd=base,env=env)
            while control.poll() is None:
                if process.poll() is not None:raise RuntimeError("Visual worker exited")
                time.sleep(.1)
            if control.returncode:raise RuntimeError("Hybrid controller failed")
        finally:
            for child in (control,process):
                if child and child.poll() is None:
                    child.terminate()
                    try:child.wait(timeout=5)
                    except subprocess.TimeoutExpired:child.kill();child.wait()
if __name__=="__main__":main()
