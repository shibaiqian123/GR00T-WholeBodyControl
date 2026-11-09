"""Fail-closed validation for the WB-WAM 31->66 inference service."""

import argparse
import json
import urllib.request


def validate(info):
    expected = {
        "state_dim": 31,
        "action_shape": [50, 66],
        "token_shape": [50, 64],
        "native_action_shape": [32, 66],
        "native_control_hz": 20,
        "control_hz": 50,
        "time_adapter": "zero_order_hold_latest_20hz_row_on_50hz_ticks",
        "token_interpolation": False,
        "rtc_mode": "disabled",
        "rtc_enabled": False,
        "gripper_output_units": "motor_output_radians",
        "gripper_label_mode": "continuous",
    }
    for key, value in expected.items():
        if info.get(key) != value:
            raise ValueError(f"server {key}={info.get(key)!r}, expected {value!r}")
    for key in (
        "checkpoint_sha256",
        "config_sha256",
        "statistics_sha256",
        "text_cache_sha256",
        "text_manifest_sha256",
        "task_sha256",
    ):
        value = info.get(key)
        if not isinstance(value, str) or len(value) != 64:
            raise ValueError(f"invalid or missing resource hash: {key}")
    return info


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8014)
    args = parser.parse_args()
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(f"http://{args.host}:{args.port}/info", timeout=10) as response:
        info = validate(json.load(response))
    print("WBWAM_SERVER_CHECK_OK " + json.dumps(info, sort_keys=True))


if __name__ == "__main__":
    main()
