"""Loopback WB-WAM inference service. No robot or motor publishers."""

from __future__ import annotations

import argparse
import asyncio
import json
import socket
import time
import uuid

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import uvicorn

from latency_trace import mark, stamp


def create_app(policy, action_exec_horizon: int = 10, *, enable_preview_source: bool = False):
    if not 1 <= action_exec_horizon < 50:
        raise ValueError("action_exec_horizon must be 1..49")
    resources = dict(policy.info, execution_mode="async")
    app = FastAPI()
    gpu_lock = asyncio.Lock()
    owner = None
    preview_latest = None

    @app.get("/health")
    async def health():
        return {"status": "ok", "motor_interfaces": False}

    @app.get("/info")
    async def info():
        return dict(
            resources,
            action_exec_horizon_default=action_exec_horizon,
            control_hz=50,
            timing="229_client_consumption_not_motor_ack",
            rtc_supported=True,
            rtc_enabled=False,
            image_encodings=["png", "rgb8_base64"],
            reply_formats=["full", "actions_only_v1"],
            preview_source_enabled=enable_preview_source,
        )

    @app.get("/preview-observation")
    async def preview_observation(after: str = ""):
        if not enable_preview_source:
            return {"enabled": False, "active": False}
        if owner is None or preview_latest is None:
            return {"enabled": True, "active": False}
        age = time.monotonic() - preview_latest["received_mono"]
        if age > 5:
            return {"enabled": True, "active": False, "source_age_seconds": age}
        result = {
            "enabled": True,
            "active": True,
            "key": preview_latest["key"],
            "source_age_seconds": age,
            "resources": resources,
            "session": preview_latest["session"],
        }
        if after != preview_latest["key"]:
            result.update(
                observation=preview_latest["observation"],
                request_id=preview_latest["request_id"],
            )
        return result

    @app.websocket("/ws")
    async def ws(socket_: WebSocket):
        nonlocal owner, preview_latest
        await socket_.accept()
        if owner is not None:
            await socket_.close(code=1013, reason="An inference client is already connected")
            return
        owner = socket_
        preview_latest = None
        session = None
        chunk_id = None
        seen: set[str] = set()
        busy = False
        try:
            while True:
                wire = await socket_.receive_text()
                received = stamp()
                request = json.loads(wire)
                trace_id = request.get("request_id", "unbound")
                mark(trace_id, "server_received", at=received)
                mark(trace_id, "server_decoded")
                if request.get("op") != "infer":
                    raise ValueError("Expected infer")
                request_id = request.get("request_id")
                request_session = request.get("session")
                if busy or not isinstance(request_id, str) or request_id in seen:
                    raise ValueError("Duplicate, missing, or concurrent request_id")
                if not isinstance(request_session, str):
                    raise ValueError("Missing session")
                if session is None:
                    session = request_session
                if request_session != session:
                    raise ValueError("Session changed within connection")
                if request.get("chunk_id") != chunk_id:
                    raise ValueError("Unknown previous chunk")
                consumed = request.get("consumed")
                delay = request.get("delay")
                if type(consumed) is not int or type(delay) is not int or not (0 <= consumed < 50 and 0 <= delay < 50):
                    raise ValueError("Invalid consumption/delay")
                if chunk_id is None and consumed != 0:
                    raise ValueError("First request consumed must be zero")
                if chunk_id is not None and consumed < action_exec_horizon:
                    raise ValueError("Replan before configured action_exec_horizon")
                observation = request.get("observation")
                if not isinstance(observation, dict):
                    raise ValueError("Observation missing")
                seen.add(request_id)
                busy = True
                if enable_preview_source:
                    preview_latest = {
                        "key": f"{session}:{request_id}",
                        "session": session,
                        "request_id": request_id,
                        "observation": observation,
                        "received_mono": time.monotonic(),
                    }
                try:
                    async with gpu_lock:
                        mark(request_id, "predict_start")
                        result = await asyncio.to_thread(policy.predict, observation, trace_id=request_id)
                    chunk_id = uuid.uuid4().hex
                    reply_format = request.get("reply_format", "full")
                    if reply_format not in ("full", "actions_only_v1"):
                        raise ValueError("Unsupported reply format")
                    if reply_format == "actions_only_v1":
                        result = {k: v for k, v in result.items() if k != "normalized"}
                        result.update(reply_format=reply_format, normalized_validated=True)
                    reply = dict(
                        result,
                        ok=True,
                        session=session,
                        request_id=request_id,
                        chunk_id=chunk_id,
                        observation_id=observation.get("observation_id"),
                        observed_at=observation.get("observed_at"),
                        window_offset=consumed,
                        estimated_delay=delay,
                        resources=resources,
                    )
                    mark(request_id, "reply_send_start")
                    await socket_.send_json(
                        reply
                    )
                finally:
                    busy = False
        except WebSocketDisconnect:
            pass
        except Exception as exc:
            try:
                await socket_.send_json({"ok": False, "session": session, "error": str(exc)})
            except Exception:
                pass
        finally:
            preview_latest = None
            owner = None
            try:
                await socket_.close()
            except Exception:
                pass
    return app


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--checkpoint", default=None)
    parser.add_argument("--ckpt-step", type=int, default=None)
    parser.add_argument("--wbwam-root", required=True)
    parser.add_argument("--text-cache", required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8014)
    parser.add_argument("--action-exec-horizon", type=int, default=10)
    parser.add_argument("--num-inference-steps", type=int, default=10)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--enable-preview-source", action="store_true")
    args = parser.parse_args(argv)
    if (args.checkpoint is None) == (args.ckpt_step is None):
        parser.error("set exactly one of --checkpoint or --ckpt-step")
    if args.ckpt_step is not None:
        if args.ckpt_step <= 0:
            parser.error("ckpt-step must be positive")
        args.checkpoint = f"step_{args.ckpt_step:06d}.pt"
    if args.num_inference_steps <= 0:
        parser.error("num-inference-steps must be positive")
    return args


def main():
    args = parse_args()
    with socket.socket() as probe:
        probe.bind((args.host, args.port))
    from model_adapter import Policy
    policy = Policy(
        run_dir=args.run_dir,
        checkpoint=args.checkpoint,
        wbwam_root=args.wbwam_root,
        text_cache=args.text_cache,
        num_inference_steps=args.num_inference_steps,
        seed=args.seed,
    )
    print("WBWAM_READY " + json.dumps(policy.info), flush=True)
    uvicorn.run(
        create_app(policy, args.action_exec_horizon, enable_preview_source=args.enable_preview_source),
        host=args.host,
        port=args.port,
        ws_max_size=16 * 1024 * 1024,
    )


if __name__ == "__main__":
    main()
