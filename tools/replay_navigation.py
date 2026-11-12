"""Replay timestamped navigation inputs without simulator or robot control access."""

import argparse
import gzip
import json
from pathlib import Path
import sys

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "controllers/sensor_capture"))

from navigation_core.contracts import MotionObservation, NavigationObservation
from navigation_core.session import NavigationSession
from navigation_core.terrain import TerrainGrid


def replay(run_directory, until_s=None):
    directory = Path(run_directory)
    config = json.loads((directory / "config_snapshot.json").read_text(encoding="utf-8"))
    grid = TerrainGrid(config["field_size_m"], config["navigation"]["grid_resolution_m"], config["navigation"])
    session = NavigationSession(config, [*config["robot_position_m"][:2], 0.0], grid)
    mismatches, count = 0, 0
    sensor_index = 0
    with open_log(directory, "navigation_inputs") as stream, open_log(directory, "sensor_observations") as sensors:
        for line in stream:
            record = json.loads(line)
            if until_s is not None and record["timestamp_s"] > until_s:
                break
            motion = MotionObservation(**record["motion"])
            if count:
                session.propagate(MotionObservation(**record.get("propagation_motion", record["motion"])), record["dt_s"])
            value = record.get("observation")
            if record.get("observation_index") is not None:
                if record["observation_index"] != sensor_index:
                    raise ValueError("Observation reference is out of order.")
                value = json.loads(next(sensors))
                sensor_index += 1
            if value is not None:
                session.observe(NavigationObservation(value["timestamp_s"], value["frame_id"], value["points"],
                                                      value["valid"], value["source"], value["metadata"]))
            command = session.tick(record["timestamp_s"], motion, record["cancelled"])
            values = (command.linear_x_m_s, command.angular_z_rad_s)
            if record["status"] != session.status or any(abs(a-b) > 1e-8 for a, b in zip(values, record["command"])):
                mismatches += 1
            count += 1
    return {"passed": mismatches == 0 and count > 0, "frames": count, "mismatched_frames": mismatches,
            "result": session.status, "control_output": "recorded_values_only"}, session


def open_log(directory, name):
    compressed = directory / (name + ".jsonl.gz")
    if compressed.exists():
        return gzip.open(compressed, "rt", encoding="utf-8")
    return (directory / (name + ".jsonl")).open(encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    args = parser.parse_args()
    result, _ = replay(args.run_directory)
    (args.run_directory / "replay_verification.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
