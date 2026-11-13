"""Build, open or verify the Lite2 Webots sensor lab."""

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import uuid
from datetime import datetime
from pathlib import Path


PROJECT = Path(__file__).resolve().parent


def find_webots():
    roots = [os.environ.get("WEBOTS_HOME"), "F:/Webots", "C:/Program Files/Webots"]
    for root in roots:
        if root:
            executable = Path(root) / "msys64" / "mingw64" / "bin" / "webots.exe"
            if executable.is_file():
                return executable
    raise RuntimeError("Set WEBOTS_HOME to the installed Webots directory.")


def verify_capture(output, scene_id):
    import numpy as np
    from PIL import Image

    state = json.loads((output / "state.json").read_text(encoding="utf-8"))
    config = json.loads((PROJECT / "config" / (scene_id + ".json")).read_text(encoding="utf-8"))
    depth = np.load(output / "depth_m.npy", allow_pickle=False)
    camera = config["camera"]
    if depth.shape != (camera["height"], camera["width"]):
        raise RuntimeError("Unexpected depth dimensions.")
    if state["robot_mode"] != "fixed_base_" + scene_id:
        raise RuntimeError("Unexpected robot mode.")
    joints = state["joint_positions_rad"]
    if set(joints) != set(config["joint_positions_rad"]):
        raise RuntimeError("Joint feedback does not match the supplied model.")
    max_error = max(abs(joints[name] - position) for name, position in config["joint_positions_rad"].items())
    if not math.isfinite(max_error) or max_error > 0.08:
        raise RuntimeError("Joint pose did not settle: " + str(max_error))
    center_depth = state["center_depth_m"]
    expected_depth = None
    if scene_id == "sensor_lab":
        robot_x = config["robot_position_m"][0]
        board = config["calibration_board"]
        expected_depth = board["position_m"][0] - board["size_m"][0] / 2 - robot_x - camera["mount_position_m"][0]
        if center_depth is None or abs(center_depth - expected_depth) > 0.03:
            raise RuntimeError(f"Camera geometry mismatch: expected {expected_depth}, measured {center_depth}.")
    if state["depth_valid_fraction"] < 0.10:
        raise RuntimeError("Too few valid depth pixels in the selected scene.")
    for filename in ["rgb.png", "depth_preview.png", "scene.png"]:
        if (output / filename).stat().st_size < 100:
            raise RuntimeError("Missing or empty image: " + filename)
    with Image.open(output / "rgb.png") as image:
        if image.size != (camera["width"], camera["height"]):
            raise RuntimeError("Unexpected RGB dimensions.")
        if np.asarray(image).std() < 1:
            raise RuntimeError("The RGB frame is blank.")
    imu = state["imu"]
    for name in ["rpy_rad", "angular_velocity_rad_s", "accelerometer_m_s2"]:
        if len(imu[name]) != 3 or not all(math.isfinite(value) for value in imu[name]):
            raise RuntimeError("Invalid inertial sensor output: " + name)
    result = {
        "result": "passed", "scope": "fixed_base_model_sensors_and_course_scene_load" if scene_id == "ruins_reference" else "fixed_base_model_and_sensors",
        "scene_id": scene_id,
        "joint_count": len(joints), "max_joint_error_rad": max_error,
        "center_depth_m": center_depth, "expected_depth_m": expected_depth,
        "depth_valid_fraction": state["depth_valid_fraction"],
    }
    (output / "verification.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", choices=["sensor_lab", "ruins_reference"], default="sensor_lab")
    parser.add_argument("--check", action="store_true", help="Capture sensors, verify the selected scene and exit Webots.")
    parser.add_argument("--navigate", action="store_true", help="Run the simulated scan-to-map SLAM and A* navigation loop.")
    parser.add_argument("--batch", action="store_true", help="Run navigation in fast mode and close Webots on completion.")
    args = parser.parse_args()
    if args.navigate and args.scene != "ruins_reference":
        parser.error("--navigate requires --scene ruins_reference.")
    if args.navigate and args.check:
        parser.error("--navigate runs interactively and cannot be combined with --check.")
    if args.batch and not args.navigate:
        parser.error("--batch requires --navigate.")
    python = PROJECT / ".venv" / "Scripts" / "python.exe"
    if not python.is_file():
        raise RuntimeError("Create .venv and install requirements.txt first.")
    if Path(sys.executable).resolve() != python.resolve():
        return subprocess.run([str(python), str(Path(__file__).resolve()), *sys.argv[1:]]).returncode

    webots = find_webots()
    subprocess.run([str(python), str(PROJECT / "tools" / "prepare_model.py"), "--scene", args.scene], check=True)
    run_name = datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:6]
    output = PROJECT / "runs" / run_name
    output.mkdir(parents=True)
    shutil.copy2(PROJECT / "config" / (args.scene + ".json"), output / "config_snapshot.json")
    shutil.copy2(PROJECT / "config" / "lite2_velocity_adapter.json", output / "lite2_velocity_adapter_snapshot.json")
    source_snapshot = output / "source_snapshot"
    source_files = [
        "run.py", "requirements.txt", "tools/prepare_model.py",
        "controllers/sensor_capture/sensor_capture.py", "controllers/sensor_capture/navigation.py",
    ]
    source_files.extend(
        str(path.relative_to(PROJECT)).replace("\\", "/")
        for path in sorted((PROJECT / "controllers" / "sensor_capture" / "navigation_core").rglob("*.py"))
    )
    for relative in source_files:
        target = source_snapshot / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(PROJECT / relative, target)
    environment = os.environ.copy()
    environment["LITE2_CAPTURE_DIR"] = str(output)
    environment["LITE2_CHECK_MODE"] = "1" if args.check else "0"
    environment["LITE2_NAVIGATE"] = "1" if args.navigate else "0"
    environment["LITE2_BATCH_MODE"] = "1" if args.batch else "0"
    environment["LITE2_CONFIG_PATH"] = str(PROJECT / "config" / (args.scene + ".json"))
    environment["LITE2_CANCEL_FILE"] = str(output / "cancel_navigation")
    command = [str(webots), "--stdout", "--stderr", "--port=12360"]
    if args.check or args.batch:
        command.extend(["--batch", "--minimize", "--mode=fast"])
    else:
        command.append("--mode=realtime")
    command.append(str(PROJECT / "worlds" / (args.scene + ".wbt")))
    print("Run output:", output, flush=True)
    with (output / "webots.log").open("w", encoding="utf-8") as log:
        process = subprocess.Popen(command, env=environment, stdout=log, stderr=subprocess.STDOUT)
        try:
            exit_code = process.wait(timeout=180 if args.check else 600 if args.batch else None)
        except (subprocess.TimeoutExpired, KeyboardInterrupt):
            subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"], capture_output=True)
            process.wait()
            raise
    error_path = output / "controller_error.txt"
    if error_path.exists():
        raise RuntimeError(error_path.read_text(encoding="utf-8"))
    if exit_code != 0:
        raise RuntimeError("Webots exited with an error. See " + str(output / "webots.log"))
    if args.check:
        verify_capture(output, args.scene)
    if args.batch:
        summary = json.loads((output / "navigation_summary.json").read_text(encoding="utf-8"))
        print(json.dumps(summary, indent=2))
        return 0 if summary["result"] == "goal_reached" else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
