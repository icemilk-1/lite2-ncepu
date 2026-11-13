"""Diagnostic wrapper for seed 43 randomized course run (no project changes).

Monkey-patches NavigationSession._replan and ._recovery_command to dump the
planner + recovery decisions, then runs run_mujoco.main() with the same argv
run_monte_carlo.py would use for seed 43.
"""
import math
import sys
from pathlib import Path

PROJECT = Path(r"F:\airace\lite2_sim")
sys.path.insert(0, str(PROJECT))
sys.path.insert(0, str(PROJECT / "controllers" / "sensor_capture"))

import numpy as np  # noqa: E402

from navigation_core import planner as P  # noqa: E402
from navigation_core import session as S  # noqa: E402

_orig_replan = S.NavigationSession._replan
_orig_recovery_command = S.NavigationSession._recovery_command


def _dbg(msg):
    print("DBG " + msg, flush=True)


_dumped = [0]


def patched_replan(self, now):
    _orig_replan(self, now)
    g = self.grid
    cell = g.world_to_cell(*self.pose[:2])
    if cell is None:
        _dbg("t=%.2f status=%s reason=%s pose_out_of_map" % (
            now, self.plan.status, self.plan.reason))
        return
    column, row = cell
    values = g.log_odds
    known_free = values <= -0.18
    occupied = values >= self.settings["occupied_log_odds_threshold"]
    unknown = ~(known_free | occupied)
    try:
        blocked = g.inflated(self.settings["robot_radius_m"], self.settings["occupied_log_odds_threshold"])
        blocked24 = g.inflated(0.24, self.settings["occupied_log_odds_threshold"])
    except Exception as exc:  # noqa: BLE001
        blocked = blocked24 = None
        _dbg("inflated error: %r" % (exc,))
    start_blocked = blocked is not None and bool(blocked[row, column])
    start_blocked24 = blocked24 is not None and bool(blocked24[row, column])
    reach_count = None
    if blocked24 is not None and known_free[row, column] and not blocked24[row, column]:
        try:
            costs, _ = P._reachable_costs(g, (column, row), known_free & ~blocked24, blocked24)
            reach_count = len(costs)
        except Exception as exc:  # noqa: BLE001
            reach_count = "err:%s" % (exc,)
    _dbg(("t=%.2f status=%s reason=%s radius=%.2f pose=(%.3f,%.3f,%.3f) cell=(%d,%d) "
          "lo=%.2f blocked=%s blocked24=%s free=%d unknown=%d reach24=%s goal=%s") % (
        now, self.plan.status, self.plan.reason,
        self.settings.get("planning_radius_m", self.settings["robot_radius_m"]),
        self.pose[0], self.pose[1], self.pose[2], column, row,
        float(values[row, column]), start_blocked, start_blocked24,
        int(known_free.sum()), int(unknown.sum()), reach_count,
        self.goal if self.goal is not None else "None"))
    if _dumped[0] < 1:
        _dumped[0] += 1
        _dump_window(g, known_free, occupied, unknown, blocked, blocked24, column, row)


def _dump_window(g, known_free, occupied, unknown, blocked, blocked24, column, row, half=22):
    hazard = getattr(g, "terrain_hazard", None)
    discontinuity = getattr(g, "height_discontinuities", None)
    r0, r1 = max(0, row - half), min(g.height, row + half + 1)
    c0, c1 = max(0, column - half), min(g.width, column + half + 1)
    _dbg("window rows %d..%d cols %d..%d (x %.2f..%.2f, y %.2f..%.2f), legend: "
         "@=start #=occupied X=blocked24(no-occ) .=free&open ?=unknown H=hazard D=height-discont" % (
             r0, r1 - 1, c0, c1 - 1, g.x_min + c0 * g.resolution,
             g.x_min + (c1 - 1) * g.resolution, g.y_min + r0 * g.resolution,
             g.y_min + (r1 - 1) * g.resolution))
    for rr in range(r1 - 1, r0 - 1, -1):
        line = []
        for cc in range(c0, c1):
            if rr == row and cc == column:
                ch = "@"
            elif hazard is not None and bool(hazard[rr, cc]):
                ch = "H"
            elif discontinuity is not None and bool(discontinuity[rr, cc]):
                ch = "D"
            elif occupied[rr, cc]:
                ch = "#"
            elif blocked24 is not None and blocked24[rr, cc]:
                ch = "X"
            elif known_free[rr, cc]:
                ch = "."
            else:
                ch = "?"
            line.append(ch)
        _dbg("r%03d %s" % (rr, "".join(line)))
    _dbg("end window")


S.NavigationSession._replan = patched_replan


def patched_recovery_command(self, linear, angular, now):
    cmd = _orig_recovery_command(self, linear, angular, now)
    reason = self.recovery.get("reason") if self.recovery is not None else None
    attempts = self.recovery.get("attempts") if self.recovery is not None else None
    _dbg("recov t=%.2f req=(%.3f,%.3f) got=(%.3f,%.3f) reason=%s attempts=%s" % (
        now, linear, angular, cmd.linear_x_m_s, cmd.angular_z_rad_s, reason, attempts))
    return cmd


S.NavigationSession._recovery_command = patched_recovery_command

if __name__ == "__main__":
    sys.argv = ["run_mujoco.py", "--scene", "ruins_reference", "--course",
                "--headless", "--seed", "43", "--randomize", "sim2real_v1"]
    import run_mujoco
    run_mujoco.main()