"""Generate simulation-only limits from the matched URDF; never starts control."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]

def make_config(root=ROOT):
    urdf = root / "gear_sonic/data/robot_model/model_data/g1/g1_29dof_with_hand.urdf"
    mapping = root / "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/policy_parameters.hpp"
    block = mapping.read_text().split("g1_action_scale = {", 1)[1].split("};", 1)[0]
    names = re.findall(r"//\s*(\w+_joint)", block)
    if len(names) != 29 or len(set(names)) != 29:
        raise ValueError("Expected 29 unique hardware-order joints")
    joints = {j.attrib["name"]: j for j in ET.parse(urdf).getroot().findall("joint")}
    limits = []
    for name in names:
        node = joints[name].find("limit")
        values = [float(node.attrib[k]) for k in ("lower", "upper", "velocity")]
        if not all(math.isfinite(x) for x in values) or values[0] >= values[1] or values[2] <= 0:
            raise ValueError("Invalid URDF limits for " + name)
        limits.append(values)
    return dict(port=15558, followup="planner", units="rad", joint_order="hardware_29",
        simulation_only=True, joint_names=names, control_dt=0.02,
        q_min=[x[0] for x in limits], q_max=[x[1] for x in limits],
        max_delta=[x[2]*0.02 for x in limits],
        constraint_source="Matched G1 URDF position limits; simulation test envelope max_delta=URDF velocity*0.02s. Not a validated hardware safety limit.",
        assets={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in (urdf, mapping)})

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    config = make_config()
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with a.output.open("x") as f:
        json.dump(config, f, indent=2)
    print("SIM_CONFIG_SAVED", a.output, "NO CONTROL STARTED")

if __name__ == "__main__":
    main()
