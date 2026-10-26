"""
All-in-one tmux launcher for SONIC data collection.

Real robot mode splits one tmux window into five panes: deploy, PICO, Dex1,
USB right-eye video, and exporter. Use --dry-run to inspect commands without launching.
The four-pane layout below applies to the legacy --sim path.

Starts the full data collection stack in a single tmux session:

    Window 0 — data_collection (4 panes):
    ┌───────────────────────┬───────────────────────┐
    │ Pane 0: C++ Deploy    │ Pane 2: Data Exporter │
    │ (gear_sonic_deploy)   │ (.venv_data_collection)│
    ├───────────────────────┼───────────────────────┤
    │ Pane 1: Teleop        │ Pane 3: Camera Viewer │
    │ (.venv_teleop)        │ (.venv_data_collection)│
    └───────────────────────┴───────────────────────┘

    Window 1 — sim  (only when --sim is passed):
    ┌─────────────────────────────────────────────────┐
    │ MuJoCo Simulator (run_sim_loop.py)              │
    │ (.venv_sim)                                     │
    └─────────────────────────────────────────────────┘

Prerequisites:
    - tmux installed (sudo apt install tmux)
    - Virtual environments set up:
        bash install_scripts/install_pico.sh            -> .venv_teleop
        bash install_scripts/install_data_collection.sh -> .venv_data_collection
    - gear_sonic_deploy built (see docs)
    - For sim: .venv_sim must exist (see install instructions)

Usage (from repo root — no venv activation needed):
    python gear_sonic/scripts/launch_data_collection.py                          # real robot (default)
    python gear_sonic/scripts/launch_data_collection.py --sim                    # MuJoCo sim
    python gear_sonic/scripts/launch_data_collection.py --no-camera-viewer       # skip viewer
    python gear_sonic/scripts/launch_data_collection.py --pico-input-source isaac-teleop  # in-process CloudXR / DeviceIO
"""

from dataclasses import dataclass
from pathlib import Path
import os
import shlex
import json
import math
import shutil
import signal
import socket
import subprocess
import sys
import time


def _bootstrap_venv():
    """Re-exec with the .venv_data_collection Python if tyro is not available."""
    try:
        import tyro  # noqa: F401
        return
    except ImportError:
        pass

    repo_root = Path(__file__).resolve().parent.parent.parent
    venv_python = repo_root / ".venv_data_collection" / "bin" / "python"
    if not venv_python.exists():
        print(
            "ERROR: tyro is not installed and .venv_data_collection not found.\n"
            "  Run: bash install_scripts/install_data_collection.sh"
        )
        sys.exit(1)

    print(f"Re-launching with {venv_python} ...")
    os.execv(str(venv_python), [str(venv_python)] + sys.argv)


_bootstrap_venv()

import tyro


def _get_local_ip() -> str:
    """Best-effort detection of the PC's LAN IP address."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "unknown"


@dataclass
class DataCollectionLaunchConfig:
    """CLI config for the all-in-one data collection tmux launcher."""

    # Real robot five-terminal preset.
    root_output_dir: str = "outputs"
    """Output root forwarded to the data exporter."""
    interface: str = "enx6c1ff7b8d821"
    """Robot Ethernet interface for deployment and Dex1."""
    listen_host: str = "192.168.11.224"
    """Workstation LAN IP used by PICO."""
    headset: str = "192.168.11.228"
    """PICO LAN IP."""
    usb_camera_port: int = 15556
    """Robot USB camera TCP port (not the exporter ZMQ port)."""
    dex1_limits: str = "gear_sonic/configs/dex1_limits.g1_20261013.json"
    """Calibrated soft limits file."""
    obstruction_timeout: float = 1.0
    """Seconds of blocked closing before per-hand hold."""
    dry_run: bool = False
    """Print terminal commands without starting processes."""

    # Deployment mode
    sim: bool = False
    """Run against MuJoCo sim (deploy.sh sim) instead of real robot."""

    # C++ deploy options
    deploy_input_type: str = "zmq_manager"
    """Input type for the C++ deploy (zmq_manager, keyboard, etc.)."""

    deploy_zmq_host: str = "localhost"
    """ZMQ host for the C++ deploy to listen on."""

    deploy_checkpoint: str = ""
    """Checkpoint path for deploy.sh (e.g., 'policy/checkpoints/my_model/model_step_100000').
    Leave empty to use the deploy.sh default."""

    deploy_obs_config: str = ""
    """Observation config file for deploy.sh. Leave empty for default."""

    deploy_planner: str = ""
    """Planner model path for deploy.sh. Leave empty for default."""

    deploy_motion_data: str = ""
    """Motion data path for deploy.sh. Leave empty for default."""

    deploy_output_type: str = ""
    """Output type for deploy.sh. Leave empty for default."""

    deploy_motor_kp_scale: str = ""
    """Kp scale specification for hardware motor indices (for example, 4,10=1.5)."""

    deploy_motor_kd_scale: str = ""
    """Kd scale specification for hardware motor indices (for example, 4,10=1.5)."""

    # Teleop streamer options
    pico_manager: bool = True
    """Run pico_manager_thread_server with --manager flag."""

    pico_input_source: str = "xrt"
    """Teleop input source for pico_manager_thread_server.py (xrt or isaac-teleop)."""

    pico_vis_vr3pt: bool = False
    """Enable VR 3-point visualization on the teleop streamer."""

    pico_vis_smpl: bool = False
    """Enable SMPL visualization on the teleop streamer."""

    pico_waist_tracking: bool = False
    """Enable waist tracking on the teleop streamer."""

    # Data exporter options
    task_prompt: str = "demo"
    """Language task prompt for the data exporter."""

    dataset_name: str = ""
    """Dataset name for the data exporter. Leave empty to auto-generate from timestamp."""

    data_exporter_frequency: int = 50
    """Data collection frequency (Hz) for the data exporter."""

    record_wrist_cameras: bool = False
    """Record wrist camera streams (left_wrist, right_wrist) in the dataset."""

    text_to_speech: bool = True
    """Enable voice feedback via espeak (data exporter)."""

    # Camera viewer
    camera_viewer: bool = True
    """Start the camera viewer pane."""

    camera_host: str = "192.168.123.164"
    """Real mode: robot USB camera host. Sim mode: simulator camera host."""

    camera_port: int = 5555
    """Local recording ZMQ port; real USB port is --usb-camera-port."""


SESSION_NAME = "sonic_data_collection"


def _check_prerequisites(config: DataCollectionLaunchConfig):
    """Verify that required tools and venvs exist."""
    errors = []

    if not shutil.which("tmux"):
        errors.append("tmux is not installed. Install with: sudo apt install tmux")

    repo_root = Path(__file__).resolve().parent.parent.parent

    if not (repo_root / ".venv_teleop" / "bin" / "activate").exists():
        errors.append(
            ".venv_teleop not found. Run: bash install_scripts/install_pico.sh"
        )

    if not (repo_root / ".venv_data_collection" / "bin" / "activate").exists():
        errors.append(
            ".venv_data_collection not found. Run: "
            "bash install_scripts/install_data_collection.sh"
        )

    deploy_dir = repo_root / "gear_sonic_deploy"
    if not (deploy_dir / "deploy.sh").exists():
        errors.append(
            f"gear_sonic_deploy/deploy.sh not found at {deploy_dir}. "
            "Ensure the deploy directory is set up."
        )

    if config.sim and not (repo_root / ".venv_sim" / "bin" / "activate").exists():
        errors.append(
            ".venv_sim not found. Set up the simulation venv first "
            "(see install instructions)."
        )

    if config.pico_input_source not in {"xrt", "isaac-teleop"}:
        errors.append("--pico-input-source must be one of: xrt, isaac-teleop")

    if errors:
        print("ERROR: Prerequisites not met:\n")
        for e in errors:
            print(f"  - {e}")
        print()
        sys.exit(1)


def _kill_existing_session():
    """Kill any existing tmux session with our name."""
    subprocess.run(
        ["tmux", "kill-session", "-t", SESSION_NAME],
        capture_output=True,
    )


def _create_tmux_session():
    """Create a 4-pane tmux layout."""
    # Create detached session
    subprocess.run(
        ["tmux", "new-session", "-d", "-s", SESSION_NAME],
        check=True,
    )

    # Enable mouse support (click panes, scroll, resize)
    subprocess.run(
        ["tmux", "set-option", "-t", SESSION_NAME, "-g", "mouse", "on"],
    )

    # Bind Ctrl+\ to kill the entire session (no prefix needed)
    subprocess.run(
        ["tmux", "bind-key", "-T", "root", "C-\\", "kill-session"],
    )

    # Rename default window
    subprocess.run(
        ["tmux", "rename-window", "-t", f"{SESSION_NAME}:0", "data_collection"],
    )

    # Split into 4 panes:
    #   0 | 1
    #   -----
    #   2 | 3

    # Split horizontally: pane 0 (left) and pane 1 (right)
    subprocess.run(
        ["tmux", "split-window", "-t", f"{SESSION_NAME}:0", "-h"],
    )

    # Split left pane vertically: pane 0 (top-left) and pane 2 (bottom-left)
    subprocess.run(
        ["tmux", "split-window", "-t", f"{SESSION_NAME}:0.0", "-v"],
    )

    # Split right pane vertically: pane 1 becomes top-right, new pane 3 bottom-right
    subprocess.run(
        ["tmux", "split-window", "-t", f"{SESSION_NAME}:0.2", "-v"],
    )

    # Let all pane shells finish initialization (.bashrc, conda, etc.)
    time.sleep(5)


def _send_to_pane(pane_index: int, cmd: str, wait: float = 1.0):
    """Send a command string to a tmux pane."""
    target = f"{SESSION_NAME}:0.{pane_index}"

    subprocess.run(
        ["tmux", "send-keys", "-t", target, cmd, "C-m"],
    )
    time.sleep(wait)


def _check_pane_alive(pane_index: int) -> bool:
    """Check if a tmux pane's process is still running."""
    target = f"{SESSION_NAME}:0.{pane_index}"
    result = subprocess.run(
        ["tmux", "list-panes", "-t", target, "-F", "#{pane_dead}"],
        capture_output=True,
        text=True,
    )
    return result.stdout.strip() != "1"


def legacy_main(config: DataCollectionLaunchConfig):
    repo_root = Path(__file__).resolve().parent.parent.parent

    _check_prerequisites(config)
    _kill_existing_session()

    print("=" * 60)
    print("  SONIC Data Collection Launcher")
    print("=" * 60)
    print(f"  Mode:            {'Simulation' if config.sim else 'Real Robot'}")
    print(f"  Task prompt:     {config.task_prompt}")
    print(f"  Dataset name:    {config.dataset_name or '(auto)'}")
    print(f"  Deploy input:    {config.deploy_input_type}")
    print(f"  Teleop input:    {config.pico_input_source}")
    if config.deploy_checkpoint:
        print(f"  Checkpoint:      {config.deploy_checkpoint}")
    print(f"  Camera:          {config.camera_host}:{config.camera_port}")
    print(f"  DC frequency:    {config.data_exporter_frequency} Hz")
    print(f"  Camera viewer:   {'Yes' if config.camera_viewer else 'No'}")
    print(f"  Wrist cameras:   {'Yes' if config.record_wrist_cameras else 'No'}")
    print(f"  Text-to-speech:  {'Yes' if config.text_to_speech else 'No'}")
    print(f"  PC IP (for PICO): {_get_local_ip()}")
    print(f"  Teleop vis:      vr3pt={config.pico_vis_vr3pt} smpl={config.pico_vis_smpl}")
    print("=" * 60)

    _create_tmux_session()
    print(f"Created tmux session: {SESSION_NAME}")

    # --- Window 1 (sim only): MuJoCo Simulator ---
    if config.sim:
        subprocess.run(
            ["tmux", "new-window", "-t", SESSION_NAME, "-n", "sim"],
        )
        sim_cmd = (
            f"cd {repo_root} && "
            f"source .venv_sim/bin/activate && "
            f"python gear_sonic/scripts/run_sim_loop.py "
            f"--enable-image-publish --enable-offscreen "
            f"--camera-port {config.camera_port}"
        )
        sim_target = f"{SESSION_NAME}:sim"
        subprocess.run(
            ["tmux", "send-keys", "-t", sim_target, sim_cmd, "C-m"],
        )
        print("Starting MuJoCo simulator (window: sim)...")
        time.sleep(3.0)

        # Switch back to the data_collection window for the remaining panes
        subprocess.run(
            ["tmux", "select-window", "-t", f"{SESSION_NAME}:data_collection"],
        )

    # --- Pane 0 (top-left): C++ Deploy ---
    deploy_mode = "sim" if config.sim else "real"
    deploy_cmd = (
        f"cd {repo_root / 'gear_sonic_deploy'} && "
        f"./deploy.sh "
        f"--input-type {config.deploy_input_type} "
        f"--zmq-host {config.deploy_zmq_host} "
    )
    if config.deploy_checkpoint:
        deploy_cmd += f"--cp {config.deploy_checkpoint} "
    if config.deploy_obs_config:
        deploy_cmd += f"--obs-config {config.deploy_obs_config} "
    if config.deploy_planner:
        deploy_cmd += f"--planner {config.deploy_planner} "
    if config.deploy_motion_data:
        deploy_cmd += f"--motion-data {config.deploy_motion_data} "
    if config.deploy_output_type:
        deploy_cmd += f"--output-type {config.deploy_output_type} "
    if config.deploy_motor_kp_scale:
        deploy_cmd += f"--motor-kp-scale {config.deploy_motor_kp_scale} "
    if config.deploy_motor_kd_scale:
        deploy_cmd += f"--motor-kd-scale {config.deploy_motor_kd_scale} "
    deploy_cmd += deploy_mode

    print("Starting C++ deploy (pane 0)...")
    _send_to_pane(0, deploy_cmd, wait=3.0)

    if not _check_pane_alive(0):
        print("WARNING: C++ deploy pane may have failed to start.")

    # --- Pane 2 (bottom-left): Teleop Streamer ---
    pico_cmd = (
        f"cd {repo_root} && "
        f"source .venv_teleop/bin/activate && "
        f"python gear_sonic/scripts/pico_manager_thread_server.py "
        f"--input-source {config.pico_input_source}"
    )
    if config.pico_manager:
        pico_cmd += " --manager"
    if config.pico_vis_vr3pt:
        pico_cmd += " --vis_vr3pt"
    if config.pico_vis_smpl:
        pico_cmd += " --vis_smpl"
    if config.pico_waist_tracking:
        pico_cmd += " --waist_tracking"

    print("Starting teleop streamer (pane 2)...")
    _send_to_pane(1, pico_cmd, wait=2.0)

    # --- Pane 3 (bottom-right): Camera Viewer ---
    if config.camera_viewer:
        viewer_cmd = (
            f"cd {repo_root} && "
            f"source .venv_data_collection/bin/activate && "
            f"python gear_sonic/scripts/run_camera_viewer.py "
            f"--camera-host {config.camera_host} "
            f"--camera-port {config.camera_port}"
        )
        print("Starting camera viewer (pane 3)...")
        _send_to_pane(3, viewer_cmd, wait=2.0)

    # --- Pane 1 (top-right): Data Exporter ---
    exporter_cmd = (
        f"cd {repo_root} && "
        f"source .venv_data_collection/bin/activate && "
        f"python gear_sonic/scripts/run_data_exporter.py "
        f"--task-prompt '{config.task_prompt}' "
        f"--data-collection-frequency {config.data_exporter_frequency} "
        f"--camera-host {config.camera_host} "
        f"--camera-port {config.camera_port}"
    )
    if config.dataset_name:
        exporter_cmd += f" --dataset-name '{config.dataset_name}'"
    if config.record_wrist_cameras:
        exporter_cmd += " --record-wrist-cameras"
    if not config.text_to_speech:
        exporter_cmd += " --no-text-to-speech"

    print("Starting data exporter (pane 1)...")
    _send_to_pane(2, exporter_cmd, wait=1.0)

    # Select the data exporter pane so the user lands there for interactive input
    subprocess.run(
        ["tmux", "select-pane", "-t", f"{SESSION_NAME}:0.2"],
    )

    print()
    print("=" * 60)
    print("  All components launched!")
    print()
    print(f"  tmux session: {SESSION_NAME}")
    print()
    if config.sim:
        print("  Window 'sim':")
        print("    MuJoCo Simulator (.venv_sim)")
        print()
    print("  Window 'data_collection':")
    print("    Pane 0 (top-left):     C++ Deploy")
    print("    Pane 1 (bottom-left):  Teleop Streamer")
    print("    Pane 2 (top-right):    Data Exporter  <-- you are here")
    if config.camera_viewer:
        print("    Pane 3 (bottom-right): Camera Viewer")
    print()
    print("  ** deploy.sh (pane 0) is waiting for confirmation —")
    print("     click on pane 0 and press Enter to proceed **")
    print()
    print("  Controls:")
    print("    Ctrl+b, arrow keys  - Switch between panes")
    if config.sim:
        print("    Ctrl+b, n / p       - Next / previous window")
    print("    Ctrl+b, d           - Detach from session")
    print("    Ctrl+\\              - Kill entire session")
    print("=" * 60)

    # Attach to the session
    try:
        subprocess.run(["tmux", "attach", "-t", SESSION_NAME])
    except KeyboardInterrupt:
        pass

    # After detach/exit, offer cleanup
    result = subprocess.run(
        ["tmux", "has-session", "-t", SESSION_NAME],
        capture_output=True,
    )
    if result.returncode == 0:
        print(f"\nSession '{SESSION_NAME}' is still running.")
        print(f"  Reattach:  tmux attach -t {SESSION_NAME}")
        print(f"  Kill:      tmux kill-session -t {SESSION_NAME}")



def build_real_commands(config, repo_root):
    """Pure command construction, shared by dry-run and launch."""
    q = shlex.quote
    def python_command(venv, args):
        return (f"cd {q(str(repo_root))} && source {q(venv + '/bin/activate')} && "
                + "exec " + shlex.join(["python", *args]))

    deploy = ["./deploy.sh", "--cp", config.deploy_checkpoint or "policy/release/model",
              "--obs-config", config.deploy_obs_config or "policy/release/observation_config.yaml",
              "--input-type", config.deploy_input_type, "--zmq-host", config.deploy_zmq_host,
              "--enable-dex1-heartbeat"]
    for flag, value in [
        ("--planner", config.deploy_planner), ("--motion-data", config.deploy_motion_data),
        ("--output-type", config.deploy_output_type),
        ("--motor-kp-scale", config.deploy_motor_kp_scale),
        ("--motor-kd-scale", config.deploy_motor_kd_scale),
    ]:
        if value:
            deploy.extend([flag, value])
    deploy.append(config.interface)
    teleop = ["gear_sonic/scripts/pico_manager_thread_server.py",
              "--input-source", config.pico_input_source, "--manager", "--port", "5556",
              "--dex1-triggers"]
    for enabled, flag in [(config.pico_vis_vr3pt, "--vis_vr3pt"),
                          (config.pico_vis_smpl, "--vis_smpl"),
                          (config.pico_waist_tracking, "--waist_tracking")]:
        if enabled:
            teleop.append(flag)
    gripper = ["-m", "gear_sonic.scripts.run_dex1_trigger_control",
               "--interface", config.interface, "--limits", config.dex1_limits,
               "--travel-time", "2", "--obstruction-hold", "--obstruction-timeout",
               str(config.obstruction_timeout), "--execute"]
    video = ["-m", "gear_sonic.scripts.run_pico_usb_stereo",
             "--camera-host", config.camera_host, "--camera-port", str(config.usb_camera_port),
             "--listen-host", config.listen_host, "--headset", config.headset,
             "--view", "right", "--publish-camera-port", str(config.camera_port)]
    if config.camera_viewer:
        video.append("--preview")
    exporter = ["gear_sonic/scripts/run_data_exporter.py",
                "--camera-host", "127.0.0.1", "--camera-port", str(config.camera_port),
                "--task-prompt", config.task_prompt, "--root-output-dir", config.root_output_dir,
                "--data-collection-frequency", str(config.data_exporter_frequency)]
    if config.dataset_name:
        exporter.extend(["--dataset-name", config.dataset_name])
    if not config.text_to_speech:
        exporter.append("--no-text-to-speech")
    return [
        ("1-deploy", f"cd {q(str(repo_root / 'gear_sonic_deploy'))} && exec {shlex.join(deploy)}"),
        ("2-pico", python_command(".venv_teleop", teleop)),
        ("3-dex1", python_command(".venv_teleop", gripper)),
        ("4-video", python_command(".venv_teleop", video)),
        ("5-exporter", python_command(".venv_data_collection", exporter)),
    ]


def check_listen_address(host, port):
    """Match server reuse semantics while still rejecting an active listener."""
    with socket.socket() as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((host, port))
            sock.listen(1)
        except OSError as exc:
            raise ValueError(f"Cannot use {host}:{port}: {exc}. Nothing was launched.") from exc


def check_real_launch(config, repo_root):
    if config.record_wrist_cameras:
        raise ValueError("The USB right-eye publisher has no wrist streams yet.")
    if not config.pico_manager or config.pico_input_source != "xrt":
        raise ValueError("The real five-terminal preset requires PICO xrt manager mode.")
    if not math.isfinite(config.obstruction_timeout) or config.obstruction_timeout <= 0:
        raise ValueError("obstruction-timeout must be positive and finite")
    for port in (config.camera_port, config.usb_camera_port):
        if not 1024 <= port <= 65535:
            raise ValueError("Camera ports must be in 1024..65535")
    if config.camera_port in (5556, 5557, 13579):
        raise ValueError("Recording port conflicts with a control/video port")
    if config.dry_run:
        return
    if subprocess.run(["tmux", "has-session", "-t", SESSION_NAME], capture_output=True).returncode == 0:
        raise ValueError(f"Session already exists. Reattach with: tmux attach -t {SESSION_NAME}")
    if not Path("/sys/class/net", config.interface).exists():
        raise ValueError(f"Robot interface does not exist: {config.interface}")
    from gear_sonic.scripts.run_dex1_trigger_control import read_limits
    read_limits(repo_root / config.dex1_limits)
    conflicts = []
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit():
            continue
        try:
            args = (proc / "cmdline").read_bytes().decode(errors="replace").split(chr(0))
            if any(Path(a).name in {"g1_deploy_onnx_ref", "pico_manager_thread_server.py",
                                   "run_data_exporter.py", "run_dex1_trigger_control.py",
                                   "run_pico_usb_stereo.py"}
                   or a in {"gear_sonic.scripts.run_dex1_trigger_control",
                            "gear_sonic.scripts.run_pico_usb_stereo"} for a in args):
                conflicts.append(proc.name)
        except (OSError, PermissionError):
            pass
    if conflicts:
        raise ValueError("Stack processes already running (PIDs " + ", ".join(conflicts)
                         + "). Stop the old stack in its terminals before launching; nothing was killed.")
    for host, port in [("127.0.0.1", config.camera_port), ("0.0.0.0", 5556),
                       ("0.0.0.0", 5557), (config.listen_host, 13579)]:
        check_listen_address(host, port)


def create_five_panes(commands, tmux_prefix=("tmux",), session_name=SESSION_NAME):
    """Left column: deploy/PICO/Dex1; right column: video/exporter."""
    def run(*args, capture=False):
        return subprocess.run([*tmux_prefix, *args], check=True,
                              capture_output=capture, text=True)
    first = run("new-session", "-d", "-s", session_name, "-n", "data_collection",
                "-x", "180", "-y", "48", "-P", "-F", "#{pane_id}", capture=True).stdout.strip()
    window = f"{session_name}:data_collection"
    run("set-option", "-t", session_name, "mouse", "on")
    run("set-option", "-w", "-t", window, "remain-on-exit", "on")
    run("set-option", "-w", "-t", window, "pane-border-status", "top")
    run("set-option", "-w", "-t", window, "pane-border-format", " #{pane_title} ")
    def split(target, direction, percent):
        return run("split-window", "-d", "-t", target, direction, "-p", str(percent),
                   "-P", "-F", "#{pane_id}", capture=True).stdout.strip()
    video = split(first, "-h", 50)
    pico = split(first, "-v", 66)
    dex1 = split(pico, "-v", 50)
    exporter = split(video, "-v", 50)
    panes = [first, pico, dex1, video, exporter]
    # Build the entire layout before starting any component.
    for pane, (name, command) in zip(panes, commands):
        run("select-pane", "-t", pane, "-T", name)
        run("respawn-pane", "-k", "-t", pane, shlex.join(["bash", "-c", command]))
    run("select-pane", "-t", first)
    return panes


def main(config):
    if config.sim:
        if config.dry_run:
            raise ValueError("--dry-run is currently supported for the real five-terminal preset only")
        return legacy_main(config)
    repo_root = Path(__file__).resolve().parent.parent.parent
    if not config.dry_run:
        _check_prerequisites(config)
    check_real_launch(config, repo_root)
    commands = build_real_commands(config, repo_root)
    for name, command in commands:
        print(f"[{name}]", command, sep=chr(10), flush=True)
    if config.dry_run:
        return
    create_five_panes(commands)
    print("Five panes created in one tmux window. Click a pane or use Ctrl+b then arrows.")
    print("Deployment retains its original confirmation prompt in the top-left pane.")
    print(f"PICO Listen: {config.listen_host}; recording: 127.0.0.1:{config.camera_port}")
    print(f"Detach: Ctrl+b then d. Reattach: tmux attach -t {SESSION_NAME}")
    subprocess.run(["tmux", "attach", "-t", SESSION_NAME])


def _signal_handler(sig, frame):
    print("\nShutdown requested...")
    subprocess.run(
        ["tmux", "kill-session", "-t", SESSION_NAME],
        capture_output=True,
    )
    sys.exit(0)


if __name__ == "__main__":
    config = tyro.cli(DataCollectionLaunchConfig)
    if config.sim:
        signal.signal(signal.SIGINT, _signal_handler)
    main(config)
