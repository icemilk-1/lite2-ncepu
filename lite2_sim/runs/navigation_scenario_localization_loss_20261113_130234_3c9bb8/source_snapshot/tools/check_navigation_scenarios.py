"""Reproducible sensor loss, cancellation, timeout and temporary-obstacle checks."""

import argparse
import csv
from datetime import datetime
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import sys
import uuid

import mujoco
import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
sys.path.insert(0, str(PROJECT / "controllers/sensor_capture"))

from mujoco_navigation import configure_navigation, run_navigation
from tools.prepare_mujoco import build_scene


class Scenario:
    def __init__(self, name):
        self.name = name
        self.drop_lidar = False
        self.drop_motion = False
        self.events = []
        self.inserted = False
        self.removed = False

    def __call__(self, now, model, data, session, motion):
        if self.name == "lidar_loss":
            self.drop_lidar = now >= 5
        elif self.name == "localization_loss" and now >= 5:
            motion.valid = False
            self.drop_motion = True
        elif self.name == "cancel" and now >= 5:
            session.cancel()
        elif self.name == "dynamic_obstacle":
            if now >= 10 and not self.inserted:
                self.inserted = True
                position = [session.pose[0] + math.cos(session.pose[2]) * 1.0,
                            session.pose[1] + math.sin(session.pose[2]) * 1.0, .20]
                model.geom_pos[model.geom("scenario_obstacle").id] = position
                mujoco.mj_forward(model, data)
                self.events.append({"time_s": now, "event": "insert", "position_m": position})
            if now >= 25 and not self.removed:
                self.removed = True
                model.geom_pos[model.geom("scenario_obstacle").id] = [30, 30, .20]
                mujoco.mj_forward(model, data)
                self.events.append({"time_s": now, "event": "remove"})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=["cancel", "timeout", "lidar_loss", "localization_loss", "dynamic_obstacle"], required=True)
    args = parser.parse_args()
    config = json.loads((PROJECT / "config/sensor_lab.json").read_text(encoding="utf-8"))
    configure_navigation(config, "sensor_lab")
    if args.scenario == "timeout":
        config["navigation"]["navigation_timeout_s"] = 5
    if args.scenario == "lidar_loss":
        config["navigation"]["navigation_timeout_s"] = 8
    if args.scenario == "dynamic_obstacle":
        config["obstacles"].append({"name": "scenario_obstacle", "position_m": [30, 30, .2], "size_m": [.5, .5, .4]})
    output = PROJECT / "runs" / ("navigation_scenario_" + args.scenario + "_" + datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:6])
    output.mkdir()
    (output / "config_snapshot.json").write_text(json.dumps(config, indent=2), encoding="utf-8")
    model, data, _ = build_scene(config, "sensor_lab", output)
    (output / "versions.json").write_text(json.dumps({"python": sys.version,
        "packages": {name: importlib.metadata.version(name) for name in ("mujoco", "numpy", "Pillow", "trimesh", "pycollada")}}, indent=2), encoding="utf-8")
    sources = [PROJECT / "mujoco_navigation.py", Path(__file__), PROJECT / "tools/prepare_mujoco.py"]
    sources.extend((PROJECT / "controllers/sensor_capture/navigation_core").rglob("*.py"))
    hashes = {}
    for source in sources:
        relative = source.relative_to(PROJECT)
        target = output / "source_snapshot" / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        payload = source.read_bytes()
        target.write_bytes(payload)
        hashes[relative.as_posix()] = hashlib.sha256(payload).hexdigest()
    (output / "source_hashes.json").write_text(json.dumps(hashes, indent=2), encoding="utf-8")
    scenario = Scenario(args.scenario)
    result = run_navigation(model, data, config, output, headless=True, scenario=scenario)
    expected = {"cancel": "cancelled", "timeout": "timeout", "lidar_loss": "timeout",
                "localization_loss": "localization_lost", "dynamic_obstacle": "goal_reached"}[args.scenario]
    passed = result["result"] == expected and result["checks"]["collision_free"] and result["checks"]["stopped_for_one_second"]
    if args.scenario == "dynamic_obstacle":
        passed = passed and result["passed"] and len(scenario.events) == 2
        observed_active = False
        obstacle_id = model.geom("scenario_obstacle").id
        for path in output.glob("lidar_*.npz"):
            with np.load(path) as batch:
                if "timestamps_s" in batch:
                    active = (batch["timestamps_s"] >= 10) & (batch["timestamps_s"] < 25)
                    observed_active |= bool(np.any(batch["hit"][active] & (batch["geom_id"][active] == obstacle_id)))
        passed = passed and observed_active
    else:
        threshold = 5 + (config["navigation"]["observation_timeout_s"] + .05 if args.scenario == "lidar_loss" else .05)
        with (output / "velocity_commands.csv").open() as stream:
            rows = list(csv.DictReader(stream))
        commands_stopped = all(abs(float(row["linear_x_m_s"])) < 1e-9 and abs(float(row["angular_z_rad_s"])) < 1e-9
                               for row in rows if float(row["sim_time_s"]) >= threshold)
        passed = passed and commands_stopped
    from mujoco_capture import capture
    capture(model, data, config, "sensor_lab", output / "final_capture", result["last_motion_observation"])
    verification = {"passed": passed, "scenario": args.scenario, "expected_result": expected,
                    "actual_result": result["result"], "events": scenario.events, "output": str(output)}
    (output / "scenario_verification.json").write_text(json.dumps(verification, indent=2), encoding="utf-8")
    print(json.dumps(verification))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
