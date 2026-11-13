"""Capture and verify the fixed-fixture MuJoCo model and ideal sensors."""

import json
import math
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from navigation_core.mujoco_adapter import MujocoLidarObservationAdapter
from run import verify_capture


def render_options():
    options = mujoco.MjvOption()
    options.geomgroup[:] = [0, 0, 1, 1, 1, 0]
    options.sitegroup[:] = 0
    return options


def capture(model, data, config, scene_id, output, motion=None):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    camera = config["camera"]
    options = render_options()
    with mujoco.Renderer(model, height=camera["height"], width=camera["width"]) as renderer:
        renderer.update_scene(data, camera="front_rgb", scene_option=options)
        rgb = renderer.render().copy()
        renderer.enable_depth_rendering()
        renderer.update_scene(data, camera="front_rgb", scene_option=options)
        depth = renderer.render().copy()
    valid = np.isfinite(depth) & (depth >= camera["min_depth_m"]) & (depth <= camera["max_depth_m"])
    depth[~valid] = np.nan
    Image.fromarray(rgb).save(output / "rgb.png")
    np.save(output / "depth_m.npy", depth, allow_pickle=False)
    np.save(output / "depth_valid.npy", valid, allow_pickle=False)
    preview = np.zeros(depth.shape, dtype=np.uint8)
    preview[valid] = np.clip(depth[valid] / camera["max_depth_m"] * 255, 0, 255).astype(np.uint8)
    Image.fromarray(preview).save(output / "depth_preview.png")

    with mujoco.Renderer(model, height=1200, width=1600) as renderer:
        overview = mujoco.MjvCamera()
        overview.lookat[:] = [0, 0, 0]
        overview.distance, overview.azimuth, overview.elevation = 16, 110, -65
        if config.get("preview"):
            preview = config["preview"]
            overview.lookat[:] = preview["lookat_m"]
            overview.distance = preview["distance_m"]
            overview.azimuth = preview["azimuth_deg"]
            overview.elevation = preview["elevation_deg"]
        renderer.update_scene(data, camera=overview, scene_option=options)
        Image.fromarray(renderer.render()).save(output / "scene.png")
        renderer.update_scene(data, camera="plan_view", scene_option=options)
        Image.fromarray(renderer.render()).save(output / "top_down.png")
        overview.lookat[:] = data.body("TORSO").xpos
        overview.distance, overview.azimuth, overview.elevation = 1.3, 135, -25
        renderer.update_scene(data, camera=overview, scene_option=options)
        Image.fromarray(renderer.render()).save(output / "robot.png")
        for detail in config.get("detail_previews", []):
            overview.lookat[:] = detail["lookat_m"]
            overview.distance = detail["distance_m"]
            overview.azimuth = detail["azimuth_deg"]
            overview.elevation = detail["elevation_deg"]
            renderer.update_scene(data, camera=overview, scene_option=options)
            Image.fromarray(renderer.render()).save(output / (detail["name"] + ".png"))

    rotation = data.body("TORSO").xmat.reshape(3, 3)
    rpy = [math.atan2(rotation[2, 1], rotation[2, 2]),
           math.asin(float(np.clip(-rotation[2, 0], -1, 1))),
           math.atan2(rotation[1, 0], rotation[0, 0])]
    focal, width, height = camera["focal_length_px"], camera["width"], camera["height"]
    center_depth = float(depth[height // 2, width // 2])
    state = {
        "schema_version": 1, "source": "mujoco", "backend": "mujoco",
        "simulation_time_s": float(data.time), "robot_mode": ("velocity_level_navigation_" if motion is not None else "fixed_base_") + scene_id,
        "fixture": "free_root_welded_to_colocated_mocap_anchor", "scene_id": scene_id,
        "camera": {"frame_id": "front_camera_optical", "optical_axes": "x_right_y_down_z_forward",
                   "sensor_model": "ideal_geometry_pending_real_sensor_characterization",
                   "width": width, "height": height,
                   "K": [focal, 0, width / 2, 0, focal, height / 2, 0, 0, 1],
                   "rgb_file": "rgb.png", "depth_file": "depth_m.npy",
                   "depth_valid_file": "depth_valid.npy", "depth_unit": "m", "depth_invalid": "nonfinite",
                   "mount_position_body_m": camera["mount_position_m"],
                   "R_body_optical": [0, 0, 1, -1, 0, 0, 0, -1, 0],
                   "mount_status": camera["mount_status"]},
        "imu": {"frame_id": "base_link", "orientation_source": "mujoco_simulated_body_orientation",
                "rpy_rad": rpy, "quaternion_wxyz": data.sensor("body_orientation").data.tolist(),
                "angular_velocity_rad_s": data.sensor("body_gyro").data.tolist(),
                "accelerometer_m_s2": data.sensor("body_accelerometer").data.tolist(),
                "accelerometer_quantity": "specific_force_body_frame_including_gravity_response"},
        "joint_positions_rad": {name: float(data.qpos[model.joint(name).qposadr[0]])
                                for name in config["joint_positions_rad"]},
        "depth_valid_fraction": float(valid.mean()),
        "center_depth_m": center_depth if math.isfinite(center_depth) else None,
        "evaluation_ground_truth": {"usage": "evaluation_only",
                                    "body_position_world_m": data.body("TORSO").xpos.tolist()},
    }
    if config.get("lidar"):
        settings = config["lidar"]
        adapter = MujocoLidarObservationAdapter(
            model, data, settings, config["robot_position_m"][2] + settings["mount_xyz_m"][2])
        observation = adapter.read(data.time, output / "lidar_raw_first.npz")
        if motion is not None:
            from navigation_core.contracts import MotionObservation
            from navigation_core.terrain import project_cloud
            measured = dict(motion)
            measured["timestamp_s"] = float(data.time)
            sample = MotionObservation(**measured)
            observation = project_cloud(adapter.raw, settings, sample,
                                         config["robot_position_m"][2] + sample.ground_height_m)
        (output / "lidar_scan.json").write_text(
            json.dumps(observation.metadata, indent=2, allow_nan=False,
                       default=lambda value: value.tolist()), encoding="utf-8")
        state["lidar"] = {"frame_id": observation.frame_id, "valid": observation.valid,
                          "timestamp_s": observation.timestamp_s, "source": observation.source,
                          "data_file": "lidar_raw_first.npz", "metadata_file": "lidar_scan.json"}
    (output / "state.json").write_text(json.dumps(state, indent=2, allow_nan=False), encoding="utf-8")
    return state


def verify(model, data, config, scene_id, output, manifest):
    """Apply shared capture checks plus model, fixture and ray checks."""
    output = Path(output)
    state = json.loads((output / "state.json").read_text(encoding="utf-8"))
    checks = {}
    if (not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all() or
            not np.isfinite(data.qacc).all() or np.any(data.warning.number)):
        raise ValueError("Simulation has nonfinite values or engine warnings.")
    if data.time < 5.0 - model.opt.timestep:
        raise ValueError("The five-second settling simulation did not complete.")
    hinge_names = {model.joint(index).name for index in range(model.njnt)
                   if model.jnt_type[index] == mujoco.mjtJoint.mjJNT_HINGE}
    if hinge_names != set(config["joint_positions_rad"]) or model.nu != 12:
        raise ValueError("Expected the 12 manufacturer revolute joints and 12 actuators.")
    if model.joint("root").type[0] != mujoco.mjtJoint.mjJNT_FREE:
        raise ValueError("The model must have its floating root.")
    mass = float(model.body_mass.sum())
    if abs(mass - manifest["source_model_mass_kg"]) > 1e-6:
        raise ValueError("Model mass changed during scene assembly.")
    inertia_error = 0.0
    for name, source in manifest["bodies"].items():
        body = model.body(name)
        principal_rotation = np.zeros(9)
        mujoco.mju_quat2Mat(principal_rotation, body.iquat)
        principal_rotation = principal_rotation.reshape(3, 3)
        actual_tensor = principal_rotation @ np.diag(body.inertia) @ principal_rotation.T
        roll, pitch, yaw = source["inertial_origin_rpy_rad"]
        cx, sx, cy, sy, cz, sz = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
        origin_rotation = np.array([[cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
                                    [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
                                    [-sy, cy * sx, cy * cx]])
        expected_tensor = origin_rotation @ np.asarray(source["inertia_tensor_kg_m2"]) @ origin_rotation.T
        # MJCF text export rounds principal moments and quaternion components.
        relative_error = float(np.linalg.norm(actual_tensor - expected_tensor) / np.linalg.norm(expected_tensor))
        inertia_error = max(inertia_error, relative_error)
        if (not np.isclose(body.mass[0], source["mass_kg"]) or
                not np.allclose(body.ipos, source["inertial_origin_xyz_m"]) or
                relative_error > 1e-5):
            raise ValueError("URDF body mass, center of mass or inertia changed: " + name)
    for name, joint in manifest["joints"].items():
        actual = model.joint(name)
        if not np.allclose(actual.axis, joint["axis"]) or not np.allclose(actual.range, joint["range_rad"]):
            raise ValueError("URDF joint axis or limits changed: " + name)
        actuator = model.actuator(name + "_servo")
        if (actuator.trnid[0] != actual.id or not np.allclose(actuator.ctrlrange, joint["range_rad"]) or
                not np.allclose(actuator.forcerange, [-joint["effort_nm"], joint["effort_nm"]])):
            raise ValueError("URDF actuator effort limit changed: " + name)
    drift = float(np.linalg.norm(data.body("TORSO").xpos - config["robot_position_m"]))
    if drift > 0.005:
        raise ValueError("Fixture drift exceeds 0.005 m: " + str(drift))
    gyro = np.asarray(state["imu"]["angular_velocity_rad_s"])
    acceleration = np.asarray(state["imu"]["accelerometer_m_s2"])
    expected = -data.body("TORSO").xmat.reshape(3, 3).T @ model.opt.gravity
    if np.linalg.norm(gyro) > 0.01 or np.linalg.norm(acceleration - expected) > 0.1:
        raise ValueError("IMU steady-state direction or gravity response is incorrect.")
    valid = np.load(output / "depth_valid.npy", allow_pickle=False)
    depth = np.load(output / "depth_m.npy", allow_pickle=False)
    if not np.array_equal(valid, np.isfinite(depth)):
        raise ValueError("The depth validity mask and stored depth disagree.")
    for shape in manifest["geometry"]:
        geom = model.geom(shape["name"])
        expected_group = 3 if shape["collision"] else 4
        quaternion = np.array([1.0, 0, 0, 0])
        if shape["rotation"] is not None:
            axis = np.array(shape["rotation"][:3], dtype=float)
            mujoco.mju_axisAngle2Quat(quaternion, axis / np.linalg.norm(axis), shape["rotation"][3])
        if geom.group[0] != expected_group:
            raise ValueError("Shared scene collision group differs: " + shape["name"])
        if shape["type"] == "mesh":
            rotation = np.zeros(9)
            mujoco.mju_quat2Mat(rotation, quaternion)
            expected_vertices = np.asarray(shape["vertices_m"]) @ rotation.reshape(3, 3).T + shape["position_m"]
            mesh_id = geom.dataid[0]
            address, count = model.mesh_vertadr[mesh_id], model.mesh_vertnum[mesh_id]
            vertices = model.mesh_vert[address:address + count]
            world_vertices = vertices @ data.geom_xmat[geom.id].reshape(3, 3).T + data.geom_xpos[geom.id]
            if not np.allclose([world_vertices.min(axis=0), world_vertices.max(axis=0)],
                               [expected_vertices.min(axis=0), expected_vertices.max(axis=0)], atol=1e-5):
                raise ValueError("Shared mesh bounds differ: " + shape["name"])
        else:
            size = np.asarray(shape["size_m"])
            expected_size = size / 2 if shape["type"] == "box" else [size[0], size[1] / 2]
            if (not np.allclose(geom.pos, shape["position_m"]) or
                    not np.allclose(geom.size[:len(expected_size)], expected_size) or
                    not np.allclose(geom.quat, quaternion)):
                raise ValueError("Shared scene geometry differs: " + shape["name"])
    if config.get("lidar"):
        settings = config["lidar"]
        with np.load(output / "lidar_raw_first.npz", allow_pickle=False) as scan:
            expected_shape = (settings["number_of_layers"], settings["horizontal_resolution"])
            if scan["range_image_m"].shape != expected_shape or not state["lidar"]["valid"]:
                raise ValueError("Unexpected LiDAR dimensions or invalid projected observation.")
            valid_ranges = scan["range_image_m"][scan["valid"]]
            if not np.all((valid_ranges >= settings["min_range_m"]) &
                          (valid_ranges <= settings["max_range_m"] + 1e-5)):
                raise ValueError("Valid LiDAR readings exceed configured limits.")
            hits = scan["geom_id"][scan["hit"]]
            if np.any(model.geom_group[hits] != 3):
                raise ValueError("LiDAR includes a robot or visual-only geometry.")
            layer = settings["number_of_layers"] // 2
            vertical = np.linspace(-settings["vertical_field_of_view_rad"] / 2,
                                   settings["vertical_field_of_view_rad"] / 2, settings["number_of_layers"])[layer]
            west_wall = next(shape for shape in manifest["geometry"] if shape["name"] == "west_wall")
            wall_geom = model.geom("west_wall")
            wall_half_x = float(np.abs(data.geom_xmat[wall_geom.id].reshape(3, 3)[0]) @ wall_geom.size)
            expected_wall = (config["robot_position_m"][0] + settings["mount_xyz_m"][0]
                             - west_wall["position_m"][0] - wall_half_x)
            wall_range = float(scan["range_image_m"][layer, 0]) * math.cos(vertical)
            if abs(wall_range - expected_wall) > 0.03:
                raise ValueError("Known west wall LiDAR distance is incorrect.")
            checks["lidar_wall_error_m"] = abs(wall_range - expected_wall)
    # Write the passing result only after all backend-specific checks succeed.
    result = verify_capture(output, scene_id, config=config, report=False)
    result.update(backend="mujoco", mujoco_version=mujoco.__version__,
                  floating_root=True, actuator_count=model.nu, source_model_mass_kg=mass,
                  fixture_drift_m=drift, gyro_norm_rad_s=float(np.linalg.norm(gyro)),
                  imu_gravity_error_m_s2=float(np.linalg.norm(acceleration - expected)),
                  max_inertia_relative_error=inertia_error,
                  geometry_count=len(manifest["geometry"]), **checks)
    (output / "verification.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result
