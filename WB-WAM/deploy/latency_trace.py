"""Best-effort local UDP trace events compatible with the existing report tool."""

from __future__ import annotations

import json
import os
from pathlib import Path
import socket
import time
import uuid

HOST = os.environ.get("LATENCY_TRACE_HOST", socket.gethostname())
try:
    BOOT = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
except OSError:
    BOOT = str(uuid.uuid4())
_socket = None
_address = None
_dropped = 0


def stamp():
    return {"host": HOST, "boot": BOOT, "mono_ns": time.monotonic_ns(), "wall_ns": time.time_ns()}


def mark(trace_id, stage, *, at=None, **meta):
    global _socket, _address, _dropped
    if not os.environ.get("LATENCY_TRACE_UDP"):
        return
    try:
        if _socket is None:
            host, port = os.environ["LATENCY_TRACE_UDP"].rsplit(":", 1)
            if host != "127.0.0.1":
                raise ValueError("trace producer must target loopback")
            _address = (host, int(port))
            _socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            _socket.setblocking(False)
        event = {
            "v": 1,
            "event_id": uuid.uuid4().hex,
            "trace_id": str(trace_id),
            "stage": stage,
            "clock": at or stamp(),
            "meta": meta,
            "emitter_drops": _dropped,
        }
        _socket.sendto(json.dumps(event, separators=(",", ":"), allow_nan=False).encode(), _address)
    except Exception:
        _dropped += 1
