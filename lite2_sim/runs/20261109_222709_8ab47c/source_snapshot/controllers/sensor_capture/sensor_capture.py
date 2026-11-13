"""Capture synchronized simulator observations from the fixed-base Lite2."""

import json
import math
import os
import traceback
from pathlib import Path

import numpy as np
from controller import Supervisor
from navigation import run_navigation


PROJECT = Path(__file__).resolve().parents[2]


def main(robot):
    embedded_config = robot.getSelf().getField("customData").getSFString()
    config_path = Path(os.environ.get("LITE2_CONFIG_PATH", embedded_config or PROJECT / "config" / "sensor_lab.json"))
    config = json.loads(config_path.read_text(encoding="utf-8"))
    scene_id = config.get("scene_id", "sensor_lab")
    output = Path(os.environ.get("LITE2_CAPTURE_DIR", str(PROJECT / "runs" / "preview")))
    output.mkdir(parents=True, exist_ok=True)
    timestep = int(robot.getBasicTimeStep())

    rgb = robot.getDevice("front_rgb")
    depth_camera = robot.getDevice("front_depth")
    orientation = robot.getDevice("body_orientation")
    accelerometer = robot.getDevice("body_accelerometer")
    gyro = robot.getDevice("body_gyro")
    for sensor in [rgb, depth_camera, orientation, accelerometer, gyro]:
        sensor.enable(timestep)

    joint_sensors = {}
    for name, position in config["joint_positions_rad"].items():
        motor = robot.getDevice(name)
        motor.setVelocity(1.0)
        motor.setPosition(position)
        sensor = motor.getPositionSensor()
        sensor.enable(timestep)
        joint_sensors[name] = sensor

    captured = False
    while robot.step(timestep) != -1:
        if captured or robot.getTime() < 2.0:
            continue

        width, height = rgb.getWidth(), rgb.getHeight()
        depth = np.asarray(depth_camera.getRangeImage(), dtype=np.float32).reshape(height, width)
        valid = np.isfinite(depth) & (depth >= depth_camera.getMinRange()) & (depth <= depth_camera.getMaxRange())
        focal = width / (2 * math.tan(rgb.getFov() / 2))
        image_path = output / "rgb.png"
        if rgb.saveImage(str(image_path), 100) != 0:
            raise RuntimeError("Could not save RGB image.")
        if depth_camera.saveImage(str(output / "depth_preview.png"), 100) != 0:
            raise RuntimeError("Could not save depth preview.")
        np.save(output / "depth_m.npy", depth, allow_pickle=False)
        state = {
            "schema_version": 1,
            "source": "webots",
            "simulation_time_s": robot.getTime(),
            "robot_mode": "fixed_base_" + scene_id,
            "scene_id": scene_id,
            "camera": {
                "frame_id": "front_camera_optical",
                "sensor_model": "ideal_geometry_pending_real_sensor_characterization",
                "optical_axes": "x_right_y_down_z_forward",
                "width": width, "height": height,
                "K": [focal, 0, width / 2, 0, focal, height / 2, 0, 0, 1],
                "rgb_file": "rgb.png", "depth_file": "depth_m.npy",
                "depth_unit": "m", "depth_invalid": "nonfinite",
                "mount_position_body_m": config["camera"]["mount_position_m"],
                "R_body_optical": [0, 0, 1, -1, 0, 0, 0, -1, 0],
                "mount_status": config["camera"]["mount_status"],
            },
            "imu": {
                "frame_id": "base_link",
                "orientation_source": "simulated_inertial_unit_ground_truth",
                "rpy_rad": list(orientation.getRollPitchYaw()),
                "angular_velocity_rad_s": list(gyro.getValues()),
                "accelerometer_m_s2": list(accelerometer.getValues()),
            },
            "joint_positions_rad": {name: sensor.getValue() for name, sensor in joint_sensors.items()},
            "depth_valid_fraction": float(valid.mean()),
            "center_depth_m": float(depth[height // 2, width // 2]),
            "evaluation_ground_truth": {
                "usage": "evaluation_only",
                "body_position_world_m": list(robot.getSelf().getPosition()),
            },
        }
        # JSON uses null for an unavailable center measurement.
        if not math.isfinite(state["center_depth_m"]):
            state["center_depth_m"] = None
        (output / "state.json").write_text(
            json.dumps(state, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8",
        )
        robot.exportImage(str(output / "scene.png"), 100)
        print("LITE2_CAPTURE_READY " + str(output), flush=True)
        captured = True
        if os.environ.get("LITE2_CHECK_MODE") == "1":
            robot.step(timestep)
            robot.simulationQuit(0)
            return
        if os.environ.get("LITE2_NAVIGATE") == "1":
            summary = run_navigation(robot, config, output, timestep)
            state["robot_mode"] = "kinematic_navigation_" + scene_id
            state["navigation"] = summary
            state["evaluation_ground_truth"]["body_position_world_m"] = list(robot.getSelf().getPosition())
            (output / "state.json").write_text(
                json.dumps(state, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8",
            )
            robot.exportImage(str(output / "scene_final.png"), 100)
            if os.environ.get("LITE2_BATCH_MODE") == "1":
                robot.simulationQuit(0)
                return


if __name__ == "__main__":
    supervisor = Supervisor()
    try:
        main(supervisor)
    except Exception:
        output = Path(os.environ.get("LITE2_CAPTURE_DIR", str(PROJECT / "runs" / "preview")))
        output.mkdir(parents=True, exist_ok=True)
        (output / "controller_error.txt").write_text(traceback.format_exc(), encoding="utf-8")
        supervisor.simulationQuit(1)
        raise
