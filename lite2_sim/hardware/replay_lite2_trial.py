"""Replay recorded trial observations through the shared navigation controller."""
import argparse
import json
from pathlib import Path
import sys
import threading

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "controllers" / "sensor_capture"))
from navigation_core.contracts import NavigationObservation
from navigation_core.control import NavigationController
from navigation_core.planner import PlanResult


def replay(path):
    controller = None
    plan = None
    mismatches = []
    count = 0
    with path.open() as stream:
        for line in stream:
            record = json.loads(line)
            if record["event"] == "configuration":
                controller = NavigationController(record["settings"])
                goal = record["goal"]
                plan = PlanResult("path_found", [(0.0, 0.0), goal], goal, goal)
            elif record["event"] == "control_tick":
                if controller is None:
                    raise ValueError("Missing trial configuration")
                observation = NavigationObservation(**record["observation"])
                decision = controller.update(record["now_s"], record["pose"], plan,
                                             observation, record["elapsed_s"])
                values = [decision.command.linear_x_m_s, decision.command.angular_z_rad_s]
                count += 1
                if (decision.status != record["status"] or
                        any(abs(a-b) > 1e-9 for a, b in zip(values, record["core_command"]))):
                    mismatches.append(count)
    return dict(ticks=count, mismatches=mismatches, passed=count > 0 and not mismatches)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, nargs="?")
    parser.add_argument("--snapshot", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.snapshot:
        import numpy as np
        from lite2_live import Inputs, floor_plane, observation, TRIAL_SETTINGS
        metadata = json.loads((args.snapshot / "snapshot.json").read_text())
        inputs = Inputs.__new__(Inputs)
        inputs.depth = np.load(str(args.snapshot / "depth.npy"))
        inputs.camera = metadata["camera"]
        inputs.depth_time = metadata["depth_timestamp_s"]
        inputs.depth_stamp_s = metadata["depth_device_timestamp_s"]
        inputs.lock = threading.Lock()
        plane = floor_plane(inputs.cloud()[0])
        observed, fraction = observation(inputs, plane)
        controller = NavigationController(dict(TRIAL_SETTINGS))
        plan = PlanResult("path_found", [(0.0, 0.0), (0.5, 0.0)], (0.5, 0.0), (0.5, 0.0))
        decision = controller.update(metadata["captured_monotonic"], (0.0, 0.0, 0.0),
                                     plan, observed, 0.0)
        result = dict(ticks=1, source="recorded_rgbd_snapshot", valid_fraction=fraction,
                      obstacle_points=len(observed.points), status=decision.status,
                      command=[decision.command.linear_x_m_s, decision.command.angular_z_rad_s],
                      passed=observed.valid and decision.status == "navigating"
                      and decision.command.linear_x_m_s > 0)
    elif args.input:
        result = replay(args.input)
    else:
        parser.error("Provide events.jsonl or --snapshot DIRECTORY")
    if args.output:
        args.output.write_text(json.dumps(result, indent=2))
    print(json.dumps(result))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
