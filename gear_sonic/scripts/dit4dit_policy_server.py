"""Same-host DiT4DiT inference service. Never imports robot control or sends commands."""
import argparse
import base64
import io
import json
import time
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--checkpoint', default='steps_19000_pytorch_model.pt')
    p.add_argument('--cosmos', required=True)
    p.add_argument('--port', type=int, default=5560)
    a = p.parse_args()
    import numpy as np
    import torch
    import zmq
    from PIL import Image
    from omegaconf import OmegaConf
    from DiT4DiT.model.framework import build_framework
    from DiT4DiT.model.framework.DiT4DiT import DiT4DiT  # register framework
    from DiT4DiT.model.framework.share_tools import dict_to_namespace
    from DiT4DiT.dataloader.gr00t_lerobot.sonic_normalization import unnormalize_sonic_actions
    from gear_sonic.utils.inference.dit4dit_control import normalize_state, validate_actions
    stats = json.loads((a.run/'dataset_statistics.json').read_text())['new_embodiment']
    cfg = OmegaConf.load(a.run/'config.yaml')
    cfg.framework.cosmos25.base_model = a.cosmos
    cfg.trainer.pretrained_checkpoint = None
    model = build_framework(dict_to_namespace(OmegaConf.to_container(cfg,resolve=True)))
    sd = torch.load(a.run/'checkpoints'/a.checkpoint,map_location='cpu',weights_only=True,mmap=True)
    print(model.load_state_dict(sd,strict=True),flush=True)
    del sd
    model.eval().requires_grad_(False).to(device='cuda',dtype=torch.bfloat16)
    ctx = zmq.Context()
    sock = ctx.socket(zmq.REP)
    sock.setsockopt(zmq.LINGER,0)
    sock.bind(f'tcp://127.0.0.1:{a.port}')
    print('READY: local inference only; no robot command sockets',flush=True)
    try:
        while True:
            req = sock.recv_json()
            try:
                if req.get('op') == 'ping':
                    sock.send_json({'ok':True,'checkpoint':a.checkpoint}); continue
                observed = float(req['observed_at'])
                if not 0 <= time.monotonic()-observed <= .5:
                    raise ValueError('Stale observation')
                image = Image.open(io.BytesIO(base64.b64decode(req['image_png'],validate=True))).convert('RGB')
                image = image.resize((224,224),Image.Resampling.BILINEAR)
                example = dict(image=[image],lang=req['lang'],state=normalize_state(req['state'],stats['state']))
                torch.manual_seed(42)
                torch.cuda.synchronize(); start=time.monotonic()
                with torch.inference_mode():
                    norm = validate_actions(model.predict_action([example])['normalized_actions'][0])
                torch.cuda.synchronize()
                actions = validate_actions(unnormalize_sonic_actions(norm,stats['action']))
                result = dict(ok=True,generation=req['generation'],request_id=req['request_id'],
                              observed_at=observed,actions=actions.tolist(),normalized=norm.tolist(),
                              out_of_range_fraction=float((abs(norm)>1).mean()),
                              inference_seconds=time.monotonic()-start)
            except Exception as exc:
                result = dict(ok=False,error=f'{type(exc).__name__}: {exc}',
                              generation=req.get('generation'),request_id=req.get('request_id'))
            sock.send_json(result)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close();ctx.term()

if __name__ == '__main__':
    main()
