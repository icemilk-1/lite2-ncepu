"""Open or verify the Lite2 MuJoCo model, shared course and ideal sensors."""

import argparse
from datetime import datetime
import importlib.metadata
import json
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
    args = parser.parse_args()
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
        config = json.loads((PROJECT / "config" / (args.scene + ".json")).read_text(encoding="utf-8"))
        (output / "config_snapshot.json").write_text(json.dumps(config, indent=2), encoding="utf-8")
        (output / "versions.json").write_text(
            json.dumps({"python": sys.version, "executable": sys.executable, "packages": versions}, indent=2), encoding="utf-8")
        files = ["run_mujoco.py", "run.py", "mujoco_capture.py", "requirements-mujoco.txt",
                 "tools/prepare_mujoco.py", "tools/prepare_model.py", "tools/scene_geometry.py",
                 "tools/reference_geometry.py",
                 "controllers/sensor_capture/navigation_core/mujoco_adapter.py",
                 "controllers/sensor_capture/navigation_core/contracts.py"]
        for relative in files:
            target = output / "source_snapshot" / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(PROJECT / relative, target)
        model, data, manifest = build_scene(config, args.scene, output)
        (output / "model_manifest.json").write_text(
            json.dumps(manifest, indent=2), encoding="utf-8")
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
        result = verify(model, data, config, args.scene, output, manifest)
        print(json.dumps(result, indent=2), flush=True)
        if not args.check:
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
        print(error, file=sys.stderr)
        print("Failed run:", output, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
