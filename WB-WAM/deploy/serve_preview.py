"""Independent-GPU WB-WAM future-video preview. No action or motor publishers."""

from __future__ import annotations

import argparse
import asyncio
import base64
from contextlib import asynccontextmanager
import io
import json
import os
import time
import urllib.parse
import urllib.request

from fastapi import FastAPI
from fastapi.responses import HTMLResponse
import uvicorn


def fetch(url):
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(url, timeout=5) as response:
        return json.load(response)


def jpeg(image):
    buffer = io.BytesIO()
    image.convert("RGB").resize((224, 224)).save(buffer, format="JPEG", quality=85)
    return "data:image/jpeg;base64," + base64.b64encode(buffer.getvalue()).decode()


PAGE = """<!doctype html><html lang='zh'><meta charset='utf-8'><title>WB-WAM 预览</title>
<style>body{background:#111827;color:#e5e7eb;font:16px system-ui;margin:30px}main{display:flex;gap:24px;flex-wrap:wrap}section{flex:1;min-width:280px}img{width:100%;max-width:560px;background:#000}#status{color:#fbbf24}pre{white-space:pre-wrap}</style>
<h1>WB-WAM 输入与未来视频</h1><p id='status'>等待输入</p><p>独立 GPU 诊断；不是动作 rollout，也不是电机执行确认。</p>
<main><section><h2>模型输入</h2><img id='input'></section><section><h2>未来视频</h2><img id='future'></section></main><pre id='meta'></pre>
<script>let key='',frames=[],i=0;async function poll(){try{let r=await fetch('/api/preview?after='+encodeURIComponent(key));let d=await r.json();document.getElementById('status').textContent=d.status+(d.error?' '+d.error:'');if(d.result){key=d.result.key;frames=d.result.frames;i=0;document.getElementById('input').src=d.result.input;document.getElementById('future').src=frames[0];document.getElementById('meta').textContent='推理 '+d.result.seconds.toFixed(3)+' 秒\n观测 '+d.result.observation_id;}}catch(e){document.getElementById('status').textContent='错误 '+e}setTimeout(poll,500)}setInterval(()=>{if(frames.length){i=(i+1)%frames.length;document.getElementById('future').src=frames[i]}},100);poll();</script></html>"""


def create_app(policy, source_url):
    state = {"status": "waiting", "error": None, "result": None}

    async def worker():
        last_key = ""
        while True:
            try:
                packet = await asyncio.to_thread(fetch, source_url + "/preview-observation?after=" + urllib.parse.quote(last_key))
                if not packet.get("active"):
                    state.update(status="source_unavailable", error=None)
                elif packet.get("observation") is not None:
                    last_key = packet["key"]
                    state.update(status="generating", error=None)
                    input_image, frames, seconds = await asyncio.to_thread(policy.predict_video, packet["observation"])
                    state.update(
                        status="ready",
                        error=None,
                        result={
                            "key": last_key,
                            "input": jpeg(input_image),
                            "frames": [jpeg(frame) for frame in frames],
                            "seconds": seconds,
                            "observation_id": packet["observation"].get("observation_id"),
                        },
                    )
            except Exception as exc:
                state.update(status="error", error=str(exc))
            await asyncio.sleep(1)

    @asynccontextmanager
    async def lifespan(_app):
        task = asyncio.create_task(worker())
        try:
            yield
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)

    app = FastAPI(lifespan=lifespan)

    @app.get("/", response_class=HTMLResponse)
    async def page():
        return PAGE

    @app.get("/api/preview")
    async def preview(after: str = ""):
        result = state["result"]
        return {"status": state["status"], "error": state["error"], "result": result if result and result["key"] != after else None}

    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--ckpt-step", type=int, required=True)
    parser.add_argument("--wbwam-root", required=True)
    parser.add_argument("--text-cache", required=True)
    parser.add_argument("--gpu-uuid", required=True)
    parser.add_argument("--source-url", default="http://127.0.0.1:8014")
    parser.add_argument("--port", type=int, default=8015)
    parser.add_argument("--num-inference-steps", type=int, default=10)
    args = parser.parse_args()
    info = fetch(args.source_url + "/info")
    if not info.get("preview_source_enabled"):
        parser.error("action server must use --enable-preview-source")
    if info.get("cuda_visible_devices") == args.gpu_uuid:
        parser.error("preview GPU must differ from action-server GPU")
    os.environ["CUDA_VISIBLE_DEVICES"] = args.gpu_uuid
    from model_adapter import Policy
    policy = Policy(
        run_dir=args.run_dir,
        checkpoint=f"step_{args.ckpt_step:06d}.pt",
        wbwam_root=args.wbwam_root,
        text_cache=args.text_cache,
        num_inference_steps=args.num_inference_steps,
        seed=42,
    )
    for key in ("checkpoint_sha256", "config_sha256", "statistics_sha256", "text_cache_sha256"):
        if info.get(key) != policy.info.get(key):
            raise ValueError(f"preview/action resource mismatch: {key}")
    print("WBWAM_PREVIEW_READY: independent GPU; no action publishers", flush=True)
    uvicorn.run(create_app(policy, args.source_url.rstrip("/")), host="127.0.0.1", port=args.port)


if __name__ == "__main__":
    main()
