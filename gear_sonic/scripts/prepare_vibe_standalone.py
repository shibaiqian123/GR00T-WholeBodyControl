"""Prepare external standalone artifacts once; never import training packages."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from gear_sonic.utils.vibe_config import read,generate,digest,load
from gear_sonic.utils.standalone.prepare_vision_trt import prepare
def main():
    p=argparse.ArgumentParser(__doc__)
    p.add_argument("--source-config",required=True)
    p.add_argument("--out",required=True)
    a=p.parse_args();src=read(a.source_config)
    out=Path(a.out).resolve();out.mkdir(parents=True,exist_ok=False)
    derived=out/"theia_float.onnx"
    prepare(src["vision_encoder"],derived)
    cfg=generate(src["bundle"],src["vision_encoder"],src["case"],".",src.get("model_root"),standalone=True)
    cfg.update(vision_trt=str(derived),vision_trt_sha256=digest(derived))
    path=out/"observation_config.yaml"
    path.write_text(json.dumps(cfg,indent=2))
    load(path)
    print(path)
if __name__=="__main__":main()
