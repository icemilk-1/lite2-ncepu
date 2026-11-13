"""Open or verify the Lite2 MuJoCo model, shared course and ideal sensors."""

import argparse
import hashlib
from datetime import datetime
import importlib.metadata
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback
import uuid


PROJECT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", choices=["sensor_lab", "ruins_reference"], default="sensor_lab")
    parser.add_argument("--check", action="store_true", help="Simulate five seconds, capture, verify and exit.")
    parser.add_argument("--navigate", action="store_true", help="Build a map online and navigate to the configured goal.")
    parser.add_argument("--explore", action="store_true", help="Map the arena online without a target and report coverage.")
    parser.add_argument("--randomize", metavar="PROFILE", help="Sample a randomization profile id or JSON path.")
    parser.add_argument("--course", action="store_true", help="Run and score the ordered course.")
    parser.add_argument("--course-stage", help="Run one configured course stage followed by a stop.")
    parser.add_argument("--config", type=Path, help="Use a separate scene configuration, including course variants.")
    parser.add_argument("--headless", action="store_true", help="Run without the interactive viewer.")
    parser.add_argument("--goal-x", type=float)
    parser.add_argument("--goal-y", type=float)
    parser.add_argument("--seed", type=int, help="Override navigation noise seed.")
    parser.add_argument("--randomization", choices=["nominal", "light", "stress"], default="nominal",
                        help="Apply a reproducible sensor and odometry disturbance profile.")
    args = parser.parse_args()
    if args.course and (args.navigate or args.check or args.explore or args.goal_x is not None or args.goal_y is not None):
        parser.error("--course is exclusive with --navigate, --explore, --check and goal overrides.")
    if args.explore and (args.navigate or args.check or args.goal_x is not None or args.goal_y is not None):
        parser.error("--explore is exclusive with --navigate, --check and goal overrides.")
    if args.course_stage and not args.course:
        parser.error("--course-stage requires --course.")
    navigation_requested = args.navigate or args.course or args.explore
    if (args.goal_x is None) != (args.goal_y is None):
        parser.error("--goal-x and --goal-y must be supplied together.")
    if args.goal_x is not None and not all(math.isfinite(value) for value in (args.goal_x, args.goal_y)):
        parser.error("Goal coordinates must be finite.")
    if args.check and args.navigate:
        parser.error("Choose --check or --navigate.")
    if not navigation_requested and (args.goal_x is not None or args.seed is not None):
        parser.error("Goal and seed overrides require --navigate.")
    if args.randomize:
        # Validate the profile before any simulator work so a typo fails fast
        # instead of leaving a half-built run directory behind.
        sys.path.insert(0, str(PROJECT / "controllers" / "sensor_capture"))
        from navigation_core.randomization import load_profile
        try:
            load_profile(args.randomize, base_dir=PROJECT)
        except (FileNotFoundError, ValueError) as error:
            parser.error(str(error))
        sys.path.remove(str(PROJECT / "controllers" / "sensor_capture"))
    python = PROJECT / ".venv-mujoco" / "Scripts" / "python.exe"
    if not python.is_file():
        raise RuntimeError("Create .venv-mujoco with standalone CPython 3.12 and install requirements-mujoco.txt.")
    if Path(sys.executable).resolve() != python.resolve():
        return subprocess.run([str(python), str(Path(__file__).resolve()), *sys.argv[1:]]).returncode

    output = PROJECT / "runs" / ("mujoco_" + datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:6])
    output.mkdir(parents=True)
    print("Run output:", output, flush=True)
    try:
        import mujoco
        sys.path.insert(0, str(PROJECT / "controllers" / "sensor_capture"))
        from mujoco_capture import capture, render_options, verify
        from tools.prepare_mujoco import build_scene, SIMULATION

        expected = {"mujoco": "3.17.0", "numpy": "2.5.4", "Pillow": "13.0.0",
                    "trimesh": "5.2.0", "pycollada": "0.9.3"}
        versions = {name: importlib.metadata.version(name) for name in expected}
        if versions != expected or sys.version_info[:2] != (3, 12):
            raise RuntimeError("Use the locked CPython 3.12 environment: " + str(versions))
        config_path = args.config or PROJECT / "config" / (args.scene + ".json")
        config = json.loads(config_path.read_text(encoding="utf-8"))
        if navigation_requested:
            from mujoco_navigation import configure_navigation
            goal = None if args.goal_x is None else (args.goal_x, args.goal_y)
            configure_navigation(config, args.scene, goal, args.seed)
            profiles = {
                "nominal": {"lidar_noise_m": 0.0, "lidar_dropout": 0.0,
                            "odom_fraction": 0.02, "yaw_noise": 0.008},
                "light": {"lidar_noise_m": 0.003, "lidar_dropout": 0.001,
                          "odom_fraction": 0.03, "yaw_noise": 0.012},
                "stress": {"lidar_noise_m": 0.01, "lidar_dropout": 0.005,
                           "odom_fraction": 0.05, "yaw_noise": 0.02},
            }
            profile = profiles[args.randomization]
            config["lidar"]["noise_m"] = profile["lidar_noise_m"]
            config["lidar"]["dropout_probability"] = profile["lidar_dropout"]
            config["navigation"]["odom_linear_noise_fraction"] = profile["odom_fraction"]
            config["navigation"]["odom_yaw_noise_rad_s"] = profile["yaw_noise"]
            config["randomization_profile"] = args.randomization
            if args.explore:
                config["navigation"]["exploration_mode"] = True
                config["navigation"]["navigation_timeout_s"] = max(
                    config["navigation"]["navigation_timeout_s"], 900.0)
            if args.randomize:
                config["randomization_profile"] = args.randomize
        if args.course:
            from navigation_core.course import validate_course
            validate_course(config.get("course", {}))
            config["course_enabled"] = True
            config["navigation"]["navigation_timeout_s"] = config["course"]["timeout_s"]
            if args.course_stage:
                stages = [s for s in config["course"]["stages"] if s["id"] == args.course_stage]
                if not stages:
                    raise ValueError("Unknown course stage: " + args.course_stage)
                stage = stages[0]
                config["robot_position_m"][:2] = stage["start_xy_m"]
                finish = {"id": "stage_finish", "kind": "finish", "timeout_s": 30,
                          "checkpoints": [dict(stage["checkpoints"][-1])]}
                finish["checkpoints"][0].pop("reverse", None)
                config["course"]["stages"] = [stage, finish]
        (output / "config_snapshot.json").write_text(json.dumps(config, indent=2), encoding="utf-8")
        (output / "versions.json").write_text(
            json.dumps({"python": sys.version, "executable": sys.executable, "packages": versions}, indent=2), encoding="utf-8")
        files = ["run_mujoco.py", "run.py", "mujoco_capture.py", "mujoco_navigation.py", "requirements-mujoco.txt",
                 "tools/prepare_mujoco.py", "tools/prepare_model.py", "tools/scene_geometry.py",
                 "tools/reference_geometry.py",
                 "controllers/sensor_capture/navigation_core/mujoco_adapter.py",
                 "controllers/sensor_capture/navigation_core/contracts.py"]
        files.extend(str(path.relative_to(PROJECT)).replace("\\", "/") for path in
                     (PROJECT / "controllers/sensor_capture/navigation_core").rglob("*.py")
                     if str(path.relative_to(PROJECT)).replace("\\", "/") not in files)
        profile = config.get("randomization_profile")
        if profile:
            candidate = Path(profile)
            if not candidate.is_file():
                candidate = PROJECT / "config" / "randomization" / (str(profile) + ".json")
            if candidate.is_file():
                try:
                    relative = str(candidate.resolve().relative_to(PROJECT.resolve())).replace("\\", "/")
                except ValueError:
                    relative = None
                if relative and relative not in files:
                    files.append(relative)
        for relative in files:
            target = output / "source_snapshot" / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(PROJECT / relative, target)
        (output / "source_hashes.json").write_text(json.dumps({
            relative: hashlib.sha256((PROJECT / relative).read_bytes()).hexdigest() for relative in files
        }, indent=2), encoding="utf-8")
        model, data, manifest = build_scene(config, args.scene, output)
        (output / "model_manifest.json").write_text(
            json.dumps(manifest, indent=2), encoding="utf-8")

        def support_height(xy):
            """Height of the environment support directly under a planar point."""
            import numpy
            origin = numpy.array([float(xy[0]), float(xy[1]), 2.0])
            direction = numpy.array([0.0, 0.0, -1.0])
            geometry = numpy.zeros(6, dtype=numpy.uint8)
            geometry[3] = 1  # environment geoms only
            distance = mujoco.mj_ray(model, data, origin, direction, geometry, True, -1, None,
                                    numpy.zeros(3))
            return float(origin[2] - distance) if distance > 0 else 0.0

        if args.course_stage:
            # A stage can start on an elevated platform; spawn the torso on the
            # local support surface instead of the scene's ground-level height,
            # otherwise the fixture weld fights the platform and drifts.
            spawn_x, spawn_y = config["robot_position_m"][:2]
            torso_height = float(config["robot_position_m"][2])
            spawn_z = support_height((spawn_x, spawn_y)) + torso_height
            config["robot_position_m"][2] = spawn_z
            data.qpos[model.jnt_qposadr[model.joint("root").id] + 2] = spawn_z
            data.mocap_pos[model.body("base_fixture").mocapid] = [spawn_x, spawn_y, spawn_z]
            mujoco.mj_forward(model, data)
            (output / "config_snapshot.json").write_text(
                json.dumps(config, indent=2), encoding="utf-8")
        steps = round(SIMULATION["settle_time_s"] / SIMULATION["physics_step_s"])
        control_steps = round(SIMULATION["control_period_s"] / SIMULATION["physics_step_s"])

        def update_targets():
            for name, target in config["joint_positions_rad"].items():
                data.ctrl[model.actuator(name + "_servo").id] = target

        for index in range(steps):
            if index % control_steps == 0:
                update_targets()
            mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        capture(model, data, config, args.scene, output)
        result = verify(model, data, config, args.scene, output, manifest,
                        pose_dependent_checks=not args.course_stage)
        print(json.dumps(result, indent=2), flush=True)
        if navigation_requested:
            from mujoco_navigation import run_navigation
            navigation = run_navigation(model, data, config, output, args.headless)
            final_output = output / "final_capture"
            capture(model, data, config, args.scene, final_output, navigation["last_motion_observation"])
            shutil.copy2(final_output / "scene.png", output / "scene_final.png")
            shutil.copy2(final_output / "top_down.png", output / "top_down_final.png")
            return 0 if navigation["passed"] else 1
        if not args.check and not args.headless:
            import mujoco.viewer
            with mujoco.viewer.launch_passive(model, data) as viewer:
                viewer.opt.geomgroup[:] = render_options().geomgroup
                viewer.opt.sitegroup[:] = 0
                viewer.cam.lookat[:] = [0, 0, 0]
                viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 16, 110, -65
                if config.get("preview"):
                    preview = config["preview"]
                    viewer.cam.lookat[:] = preview["lookat_m"]
                    viewer.cam.distance = preview["distance_m"]
                    viewer.cam.azimuth = preview["azimuth_deg"]
                    viewer.cam.elevation = preview["elevation_deg"]
                while viewer.is_running():
                    start = time.perf_counter()
                    update_targets()
                    mujoco.mj_step(model, data, nstep=control_steps)
                    viewer.sync()
                    time.sleep(max(0, SIMULATION["control_period_s"] - (time.perf_counter() - start)))
        return 0
    except Exception:
        error = traceback.format_exc()
        (output / "error.log").write_text(error, encoding="utf-8")
        (output / "verification.json").write_text(
            json.dumps({"result": "failed", "backend": "mujoco", "scene_id": args.scene,
                        "error": str(sys.exc_info()[1]), "error_log": "error.log"}, indent=2), encoding="utf-8")
        if navigation_requested and not (output / "navigation_verification.json").exists():
            (output / "navigation_verification.json").write_text(json.dumps({
                "passed": False, "result": "runtime_error", "error": str(sys.exc_info()[1]),
                "error_log": "error.log", "backend": "mujoco"}, indent=2), encoding="utf-8")
        if args.course and not (output / "course_verification.json").exists():
            (output / "course_verification.json").write_text(json.dumps({
                "passed": False, "result": "runtime_error", "error": str(sys.exc_info()[1]),
                "error_log": "error.log"}, indent=2), encoding="utf-8")
        print(error, file=sys.stderr)
        print("Failed run:", output, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
