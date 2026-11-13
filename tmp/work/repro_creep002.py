"""Offline reproduction of the dead_end_exit standstill decision.

Rebuilds the grid from a finished run's artefacts (slam_map.pgm + terrain_map.npz)
and asks the real NavigationSession for the local rollout at the recorded pose.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

PROJECT = Path(__file__).resolve().parents[2] / "lite2_sim"
sys.path.insert(0, str(PROJECT))
sys.path.insert(0, str(PROJECT / "controllers" / "sensor_capture"))

from mujoco_navigation import configure_navigation  # noqa: E402
from navigation_core.planner import PlanResult  # noqa: E402
from navigation_core.session import NavigationSession  # noqa: E402
from navigation_core.terrain import TerrainGrid  # noqa: E402
from navigation_core.contracts import NavigationObservation, MotionObservation  # noqa: E402

RUN = Path(sys.argv[1] if len(sys.argv) > 1 else
           r"F:\airace\lite2_sim\runs\mujoco_20261113_092735_751337")
POSE = (-0.25546213517043564, 0.654942666447223, -0.6351742908921776)
GOAL = (-1.11, -1.25)

config = json.loads((PROJECT / "config" / "ruins_reference.json").read_text(encoding="utf-8"))
configure_navigation(config, "ruins_reference", GOAL, 42)
settings = config["navigation"]

grid = TerrainGrid(config["field_size_m"], settings["grid_resolution_m"], settings)
meta = json.loads((RUN / "slam_map.json").read_text(encoding="utf-8"))
res = float(meta["resolution_m"])
raw = (RUN / "slam_map.pgm").read_bytes()
header, body = raw.split(b"\n255\n", 1)
width, height = [int(x) for x in header.split(b"\n")[1].split()]
image = np.frombuffer(body, dtype=np.uint8).reshape(height, width)
image = np.flipud(image)          # back to grid row order used by save_map
assert (grid.height, grid.width) == (height, width), (grid.height, grid.width, height, width)

log_odds = np.zeros(image.shape, dtype=np.float32)
log_odds[image == 250] = -1.0     # free
log_odds[image == 25] = 1.0       # occupied
np.savez_compressed  # noqa: B018  (keep import surface obvious)
terrain = np.load(RUN / "terrain_map.npz")
grid.log_odds = log_odds
grid.support_z = np.asarray(terrain["height_m"], dtype=np.float32)
grid.terrain_hazard = np.asarray(terrain["support_hazard"], dtype=bool)
grid.height_m = np.asarray(terrain["height_m"], dtype=np.float32)
grid.clearance_m = np.asarray(terrain["clearance_m"], dtype=np.float32)
grid.scan_count = 4000

replans = json.loads((RUN / "replan_history.json").read_text(encoding="utf-8"))
record = [r for r in replans if abs(r["sim_time_s"] - 193.4) < 1e-6][0]
path = [tuple(p) for p in record["poses_xy_m"]]

session = NavigationSession(config, POSE, grid)
session.settings.update(settings)
session.settings["forward_only"] = True       # dead_end cp0
session.settings["reverse_only"] = False
session.settings["alignment_threshold_rad"] = 1.15
session.settings["cross_track_gain"] = 0.0
session.settings["standstill_creep_m_s"] = 0.02
session.pose = list(POSE)
session.status = "navigating"
session.plan = PlanResult("path_found", path, GOAL, GOAL)
session.goal = GOAL
now_stamp = 193.4
session.motion = MotionObservation(now_stamp, 0.0, 0.0, 0.0, 0.0, 0.0)
session.last_observation = NavigationObservation(
    now_stamp, "front_lidar_level", np.zeros((0, 3), dtype=np.float32), True, "repro", {})

decision = session.navigator.update(now_stamp, session.pose, session.plan, session.last_observation, now_stamp)
print("navigator preferred:", decision.status, decision.command.linear_x_m_s, decision.command.angular_z_rad_s)
command = session._local_rollout(decision.command, now_stamp)
print("rollout command :", command.linear_x_m_s, command.angular_z_rad_s)
print("debug:", json.dumps(session.local_rollout_debug, indent=2))
print("desired heading:", math.atan2(
    session.navigator._lookahead_target(session.pose, path, 0.45)[1] - POSE[1],
    session.navigator._lookahead_target(session.pose, path, 0.45)[0] - POSE[0]) - POSE[2])