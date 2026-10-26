"""Entry point for running a MuJoCo simulation loop with the G1 robot model.

Parses a YAML-based WBC config via tyro CLI, instantiates the G1 robot model,
and launches the simulator (optionally with offscreen image publishing).
"""

# Optional training-asset simulation; default SONIC path below is unchanged.
# Keep this dispatch ahead of tyro and preserve run_sim_realtime.py's loop patch.
if __name__ == "__main__":
    import os, sys
    from pathlib import Path
    if "--vibe-config" in sys.argv or any(x.startswith("--vibe-config=") for x in sys.argv):
        args=[]
        for item in sys.argv[1:]:
            if item=="--vibe-config": item="--config"
            elif item.startswith("--vibe-config="): item="--config="+item.split("=",1)[1]
            args.append(item)
        root=Path(__file__).resolve().parents[2]
        python=os.environ.get("VIBE_DEPLOY_PYTHON",str(root/".venv_vibe_deploy/bin/python"))
        env=os.environ.copy()
        env["PYTHONPATH"]=str(root)
        env["PYTHONNOUSERSITE"]="1"
        print("VIBE_SIM: training Dex1, original SONIC elastic band; simulation interpreter="+python,flush=True)
        os.execve(python,[python,str(root/"gear_sonic/scripts/run_vibe_hybrid_sim.py"),*args],env)

from typing import Dict

import tyro

from gear_sonic.utils.mujoco_sim.simulator_factory import SimulatorFactory, init_channel
from gear_sonic.utils.mujoco_sim.configs import SimLoopConfig
from gear_sonic.data.robot_model.instantiation.g1 import (
    instantiate_g1_robot_model,
)
from gear_sonic.data.robot_model.robot_model import RobotModel

ArgsConfig = SimLoopConfig


class SimWrapper:
    def __init__(self, robot_model: RobotModel, env_name: str, config: Dict[str, any], **kwargs):
        self.robot_model = robot_model
        self.config = config

        init_channel(config=self.config)

        # Create simulator using factory
        self.sim = SimulatorFactory.create_simulator(
            config=self.config,
            env_name=env_name,
            **kwargs,
        )


def main(config: ArgsConfig):
    wbc_config = config.load_wbc_yaml()
    # NOTE: we will override the interface to local if it is not specified
    wbc_config["ENV_NAME"] = config.env_name

    if config.enable_image_publish:
        assert (
            config.enable_offscreen
        ), "enable_offscreen must be True when enable_image_publish is True"

    robot_model = instantiate_g1_robot_model()

    sim_wrapper = SimWrapper(
        robot_model=robot_model,
        env_name=config.env_name,
        config=wbc_config,
        onscreen=wbc_config.get("ENABLE_ONSCREEN", True),
        offscreen=wbc_config.get("ENABLE_OFFSCREEN", False),
        enable_image_publish=config.enable_image_publish,
    )
    # Start simulator as independent process
    SimulatorFactory.start_simulator(
        sim_wrapper.sim,
        as_thread=False,
        enable_image_publish=config.enable_image_publish,
        mp_start_method=config.mp_start_method,
        camera_port=config.camera_port,
    )


if __name__ == "__main__":
    config = tyro.cli(ArgsConfig)
    main(config)