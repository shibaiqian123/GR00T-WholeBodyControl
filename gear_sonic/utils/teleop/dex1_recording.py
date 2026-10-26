"""Versioned, same-host Dex1 telemetry. No hardware or network on import.

Command validity means accepted by the local DDS publisher, not motor execution.
All ages use the shared Linux monotonic clock; never use remote-host timestamps.
"""
import json
import math
import time
import uuid
from pathlib import Path

VERSION = 1
TOPIC = b"dex1_telemetry "
SIDES = ("left", "right")
PHASES = {name: i for i, name in enumerate(
    ("UNKNOWN", "WAIT_INPUTS", "OPENING", "RELEASE_TO_ARM", "READY", "FAULT_HOLD"))}


def features():
    result = {}
    for key, dtype, size in [
        ("observation.dex1_position", "float32", 2),
        ("observation.dex1_velocity", "float32", 2),
        ("action.dex1_target_position", "float32", 2),
        ("teleop.dex1_trigger", "float32", 2),
        ("teleop.dex1_requested_position", "float32", 2),
        *[("dex1." + name, "int64", 2) for name in
          ("feedback_valid", "command_valid", "trigger_valid")],
        *[("dex1." + name, "float64", 2) for name in
          ("feedback_time", "command_time", "trigger_time",
           "feedback_age", "command_age", "trigger_age")],
        ("dex1.sample_time", "float64", 1),
        ("dex1.frame_time", "float64", 1),
        ("dex1.sequence", "int64", 1),
        ("dex1.session_id", "int64", 1),
        ("dex1.control_state", "int64", 1),
    ]:
        result[key] = {"dtype": dtype, "shape": (size,),
                       "names": list(SIDES) if size == 2 else [key.split(".")[-1]]}
    return result


def modalities():
    return {group: {side + "_gripper": {
        "original_key": key, "start": i, "end": i + 1, "absolute": True}
        for i, side in enumerate(SIDES)} for group, key in (
            ("state", "observation.dex1_position"),
            ("action", "action.dex1_target_position"))}


def metadata(calibration, max_age=0.1):
    return {"version": VERSION, "calibration": calibration, "max_age_seconds": max_age,
            "clock": "same_host_monotonic_seconds", "side_order": list(SIDES),
            "position_units": "motor_output_radians", "velocity_units": "radians_per_second",
            "trigger_units": "0=open,1=closed", "phase_codes": PHASES,
            "command_semantics": "final target accepted by DDS writer; not execution acknowledgement",
            "invalid_values": "last available finite value or zero; always consult validity masks"}


def check_dataset(root, calibration, max_age=0.1, expected_features=None, expected_modalities=None, fps=None):
    """Read-only preflight; never migrate or silently append to an old schema."""
    root = Path(root)
    if not root.exists():
        return
    try:
        info = json.loads((root / "meta/info.json").read_text())
        if info.get("script_config", {}).get("dex1_recording") != metadata(calibration, max_age):
            raise ValueError("Dex1 recording version/calibration/config differs")
        for key, spec in features().items():
            actual = info["features"][key]
            if actual["dtype"] != spec["dtype"] or list(actual["shape"]) != list(spec["shape"]):
                raise ValueError("Dex1 feature mismatch: " + key)
        saved = json.loads((root / "meta/modality.json").read_text())
        for group, entries in modalities().items():
            for key, spec in entries.items():
                if saved[group].get(key) != spec:
                    raise ValueError("Dex1 modality mismatch: " + key)
        if fps is not None and info.get("fps") != fps:
            raise ValueError("Dataset FPS differs")
        if expected_features is not None:
            automatic = {"timestamp", "frame_index", "episode_index", "index", "task_index"}
            if set(info["features"])-automatic != set(expected_features)-automatic:
                raise ValueError("Dataset field names differ")
            for key,spec in expected_features.items():
                actual=info["features"][key]
                if actual["dtype"] != spec["dtype"] or list(actual["shape"]) != list(spec["shape"]) or actual.get("names") != spec.get("names"):
                    raise ValueError("Dataset field definition differs: " + key)
        if expected_modalities is not None and saved != expected_modalities:
            raise ValueError("Dataset modality definitions differ")
    except (OSError, KeyError, ValueError, TypeError) as exc:
        raise ValueError(f"Cannot append Dex1 recording to {root}: {exc}. "
                         "Use a new dataset name ending in _dex1; old data was not changed.") from exc


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def make_packet(session, sequence, now, measured, trigger, writes, controller):
    trigger = trigger or {}
    sides = {}
    for side in SIDES:
        q, rx, dq = measured.get(side, (None, -1., None))
        command = writes.get(side)
        t = trigger.get(side)
        sides[side] = {
            "q": q if finite(q) else None, "dq": dq if finite(dq) else None,
            "feedback_time": rx,
            "feedback_valid": finite(q) and finite(dq) and controller.limits[side].contains(q),
            "target": command[0] if command else None,
            "command_time": command[1] if command else -1.,
            "command_valid": command is not None,
            "trigger": t if finite(t) else None,
            "trigger_time": trigger.get("sample_time", -1.),
            "trigger_valid": trigger.get("valid") is True and not trigger.get("stop", True)
                             and finite(t) and 0 <= t <= 1,
            "requested": controller.requested.get(side) if controller.phase == "READY" else None,
        }
    return {"version": VERSION, "session": session, "seq": sequence, "time": now,
            "phase": PHASES.get(controller.phase, 0), "sides": sides}


class Publisher:
    def __init__(self, port):
        import zmq
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.SNDHWM, 5)
        self.socket.bind(f"tcp://127.0.0.1:{port}")
        self.session = uuid.uuid4().int & ((1 << 63) - 1)
        self.sequence = 0

    def publish(self, measured, trigger, writes, controller):
        import msgpack
        import zmq
        packet = make_packet(self.session, self.sequence, time.monotonic(),
                             measured, trigger, writes, controller)
        self.sequence += 1
        try:
            self.socket.send(TOPIC + msgpack.packb(packet, use_bin_type=True), flags=zmq.NOBLOCK)
        except zmq.Again:
            pass

    def close(self):
        self.socket.close()
        self.context.term()


class SampleCache:
    def __init__(self, max_age=0.1):
        self.max_age = max_age
        self.packet = None
        self.values = {}

    def accept(self, p, now):
        try:
            if p["version"] != VERSION or not finite(p["time"]) or not 0 <= now-p["time"] <= self.max_age:
                return False
            if not isinstance(p["session"], int) or not 0 <= p["session"] < 2**63:
                return False
            if not isinstance(p["seq"], int) or p["seq"] < 0:
                return False
            for s in SIDES:
                d = p["sides"][s]
                for kind in ("feedback", "command", "trigger"):
                    if type(d[kind+"_valid"]) is not bool or not finite(d[kind+"_time"]):
                        return False
            if self.packet:
                if p["time"] <= self.packet["time"]:
                    return False
                if p["session"] == self.packet["session"] and p["seq"] <= self.packet["seq"]:
                    return False
            if self.packet and p["session"] != self.packet["session"]:
                self.values.clear()
            self.packet = p
            return True
        except (KeyError, TypeError, ValueError):
            return False

    def frame(self, now):
        import numpy as np
        out = {k: np.zeros(v["shape"], dtype=v["dtype"]) for k,v in features().items()}
        p = self.packet
        out["dex1.frame_time"][0] = now
        for kind in ("feedback", "command", "trigger"):
            out["dex1."+kind+"_time"][:] = -1
            out["dex1."+kind+"_age"][:] = -1
        out["dex1.sequence"][0] = -1
        if not p:
            return out
        for key, value in (("sample_time", p["time"]), ("sequence", p["seq"]),
                           ("session_id", p["session"]), ("control_state", p.get("phase",0))):
            out["dex1."+key][0] = value
        fresh = 0 <= now-p["time"] <= self.max_age
        for i,s in enumerate(SIDES):
            d = p["sides"][s]
            for source,key in (("q","observation.dex1_position"),
                               ("dq","observation.dex1_velocity"),
                               ("target","action.dex1_target_position"),
                               ("trigger","teleop.dex1_trigger"),
                               ("requested","teleop.dex1_requested_position")):
                val = d.get(source)
                if finite(val):
                    self.values[(s,source)] = val
                out[key][i] = self.values.get((s,source),0.)
            for kind, required in (("feedback",("q","dq")), ("command",("target",)), ("trigger",("trigger",))):
                stamp = d[kind+"_time"]
                age = now-stamp if stamp >= 0 else -1.
                valid = fresh and d[kind+"_valid"] and 0 <= age <= self.max_age and all(finite(d.get(k)) for k in required)
                if kind == "trigger":
                    valid = valid and 0 <= d["trigger"] <= 1
                out["dex1."+kind+"_time"][i] = stamp
                out["dex1."+kind+"_age"][i] = age
                out["dex1."+kind+"_valid"][i] = int(valid)
        return out


class Subscriber(SampleCache):
    def __init__(self, port, max_age=0.1):
        import zmq
        super().__init__(max_age)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.LINGER,0)
        self.socket.setsockopt(zmq.RCVHWM,5)
        self.socket.setsockopt(zmq.SUBSCRIBE,TOPIC)
        self.socket.connect(f"tcp://127.0.0.1:{port}")

    def poll(self):
        import msgpack
        import zmq
        for _ in range(100):
            try:
                raw = self.socket.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            try:
                if raw.startswith(TOPIC):
                    self.accept(msgpack.unpackb(raw[len(TOPIC):],raw=False),time.monotonic())
            except (ValueError, TypeError, msgpack.ExtraData, msgpack.FormatError, msgpack.StackError):
                pass

    def close(self):
        self.socket.close()
        self.context.term()
