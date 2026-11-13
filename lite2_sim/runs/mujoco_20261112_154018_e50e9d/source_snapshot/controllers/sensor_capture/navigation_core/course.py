"""Observed-state course scheduling and independent ordered traversal scoring."""

import math

import numpy as np

from navigation_core.contracts import VelocityCommand
from navigation_core.session import NavigationSession, TERMINAL_STATES


def in_region(pose, checkpoint):
    return math.hypot(pose[0] - checkpoint["xy_m"][0],
                      pose[1] - checkpoint["xy_m"][1]) <= checkpoint["radius_m"]


def validate_course(course):
    stages = course.get("stages", [])
    if not stages or len({stage["id"] for stage in stages}) != len(stages):
        raise ValueError("Course requires stages with unique IDs.")
    for stage in stages:
        if stage["kind"] not in ("traverse", "dead_end", "reject", "finish"):
            raise ValueError("Unsupported course stage kind.")
        if not math.isfinite(stage["timeout_s"]) or stage["timeout_s"] <= 0:
            raise ValueError("Stage timeout must be positive and finite.")
        if not stage["checkpoints"]:
            raise ValueError("Stage requires checkpoints.")
        for point in stage["checkpoints"]:
            values = list(point["xy_m"]) + [point["radius_m"]]
            if len(point["xy_m"]) != 2 or not all(math.isfinite(x) for x in values) or point["radius_m"] <= 0:
                raise ValueError("Checkpoint requires finite coordinates and a positive radius.")
        if stage["kind"] in ("dead_end", "reject"):
            portal = stage["probe"]
            if (len(portal["center_xy_m"]) != 2 or not all(math.isfinite(x) for x in
                    portal["center_xy_m"] + [portal["yaw_rad"], portal["depth_m"], portal["half_width_m"]])
                    or min(portal["depth_m"], portal["half_width_m"]) <= 0):
                raise ValueError("Invalid observation probe.")
    if stages[-1]["kind"] != "finish":
        raise ValueError("Course must end with a finish stage.")


class CourseSession(object):
    """Keeps one navigation map while changing goals using estimated pose only."""

    def __init__(self, config, initial_pose, grid=None):
        self.course = config["course"]
        validate_course(self.course)
        self.navigation = NavigationSession(config, initial_pose, grid)
        self.stage_index = 0
        self.checkpoint_index = 0
        self.stage_started_at = 0.0
        self.course_started_at = None
        self.state = "initializing"
        self.events = []
        self.evidence = None
        self.probe_started_at = None
        self.completed = False
        self._set_goal(0.0)

    def __getattr__(self, name):
        return getattr(self.navigation, name)

    @property
    def stage(self):
        return self.course["stages"][self.stage_index]

    @property
    def status(self):
        return self.navigation.status

    @status.setter
    def status(self, value):
        self.navigation.status = value

    @property
    def finished(self):
        return self.completed or (self.navigation.finished and self.navigation.status != "goal_reached")

    def _event(self, now, event, **details):
        self.events.append(dict(timestamp_s=now, stage_id=self.stage["id"],
                                checkpoint_index=self.checkpoint_index, event=event,
                                estimated_pose_m_rad=list(self.pose), **details))

    def _set_goal(self, now):
        point = self.stage["checkpoints"][self.checkpoint_index]
        self.navigation.set_goal(point["xy_m"], now)
        self.navigation.settings["estimated_stop_tolerance_m"] = min(0.12, point["radius_m"] * 0.6)
        self.navigation.settings["reverse_only"] = bool(point.get("reverse", False))
        self.state = "recovering" if point.get("reverse") else "approaching"
        self._event(now, "checkpoint_started", goal_xy_m=point["xy_m"], state=self.state)

    def _observed_barrier(self, now):
        observation = self.last_observation
        if (observation is None or not observation.valid or observation.timestamp_s > now
                or now - observation.timestamp_s > self.settings["observation_timeout_s"]):
            return None
        points = np.asarray(observation.metadata.get("obstacle_points_sensor_level_m", []))
        if not len(points):
            return None
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        mount = self.lidar["mount_xy_m"]
        world = np.column_stack((self.pose[0] + c*(points[:, 0]+mount[0]) - s*(points[:, 1]+mount[1]),
                                 self.pose[1] + s*(points[:, 0]+mount[0]) + c*(points[:, 1]+mount[1])))
        probe = self.stage["probe"]
        relative = world - np.asarray(probe["center_xy_m"])
        c, s = math.cos(probe["yaw_rad"]), math.sin(probe["yaw_rad"])
        along = relative[:, 0]*c + relative[:, 1]*s
        lateral = -relative[:, 0]*s + relative[:, 1]*c
        mask = (np.abs(along) <= probe["depth_m"]) & (np.abs(lateral) <= probe["half_width_m"])
        lateral = lateral[mask]
        if len(lateral) < 4:
            return None
        if self.stage["kind"] == "dead_end":
            blocked = np.abs(lateral) < 0.20
            if blocked.sum() < 4:
                return None
            return {"reason": "observed_closed_branch", "return_count": int(blocked.sum()),
                    "observation_timestamp_s": observation.timestamp_s}
        left, right = lateral[lateral > 0], lateral[lateral < 0]
        if not len(left) or not len(right):
            return None
        width = float(left.min() - right.max())
        required = self.course["body_width_m"] + 2*self.course["clearance_margin_m"]
        if width >= required:
            return None
        return {"reason": "observed_aperture_too_narrow", "observed_width_m": width,
                "required_width_m": required, "return_count": len(lateral),
                "observation_timestamp_s": observation.timestamp_s}

    def tick(self, now, motion=None, cancelled=False):
        if self.completed:
            return VelocityCommand(0, 0, now, now+self.settings["command_timeout_s"])
        if self.course_started_at is None:
            self.course_started_at = now
            self.stage_started_at = now
        if now - self.stage_started_at >= self.stage["timeout_s"] and self.status not in TERMINAL_STATES:
            self.status = "timeout"
            self._event(now, "stage_failed", reason="stage_timeout")
        command = self.navigation.tick(now, motion, cancelled)
        if self.status != "goal_reached":
            self.state = "cancelled" if self.status == "cancelled" else ("failed" if self.finished else self.state)
            return command
        point = self.stage["checkpoints"][self.checkpoint_index]
        if not in_region(self.pose, point):
            self.status = "stalled"
            return VelocityCommand(0, 0, now, now+self.settings["command_timeout_s"])
        if self.stopped_since is None or now-self.stopped_since < self.settings["settling_stop_s"]:
            return command
        if self.stage["kind"] in ("dead_end", "reject") and self.checkpoint_index == self.stage.get("probe_checkpoint", 0) and self.evidence is None:
            self.state = "observing"
            if self.probe_started_at is None:
                self.probe_started_at = now
            self.evidence = self._observed_barrier(now)
            if self.evidence is None:
                if now-self.probe_started_at >= self.course["probe_timeout_s"]:
                    self.status = "insufficient_observations"
                    self._event(now, "stage_failed", reason="barrier_unconfirmed")
                return command
            self._event(now, "branch_closed" if self.stage["kind"] == "dead_end" else "passage_rejected",
                        evidence=self.evidence)
        self._event(now, "checkpoint_reached")
        self.checkpoint_index += 1
        if self.checkpoint_index == len(self.stage["checkpoints"]):
            self._event(now, "stage_completed")
            if self.stage_index == len(self.course["stages"])-1:
                self.state = "finished"
                self.completed = True
                return command
            self.stage_index += 1
            self.checkpoint_index = 0
            self.stage_started_at = now
            self.evidence = None
            self.probe_started_at = None
        self._set_goal(now)
        return command


class CourseEvaluator(object):
    """Scores ordered regions with independent truth samples and observed events."""

    def __init__(self, course):
        validate_course(course)
        self.course = course
        self.stage_index = 0
        self.checkpoint_index = 0
        self.results = [{"id": s["id"], "passed": False, "checkpoints": []} for s in course["stages"]]
        self.samples = []

    def record(self, now, truth_pose, velocity):
        self.samples.append((now, list(truth_pose), velocity))
        if self.stage_index == len(self.results):
            return
        stage = self.course["stages"][self.stage_index]
        checkpoint = stage["checkpoints"][self.checkpoint_index]
        if not in_region(truth_pose, checkpoint):
            return
        if checkpoint.get("reverse") and velocity >= -0.001:
            return
        self.results[self.stage_index]["checkpoints"].append({"index": self.checkpoint_index,
            "timestamp_s": now, "truth_pose_m_rad": list(truth_pose), "reverse": bool(checkpoint.get("reverse"))})
        self.checkpoint_index += 1
        if self.checkpoint_index == len(stage["checkpoints"]):
            self.results[self.stage_index]["passed"] = True
            self.stage_index += 1
            self.checkpoint_index = 0

    def summary(self, events, navigation_checks, status):
        for stage, result in zip(self.course["stages"], self.results):
            required = {"dead_end": "branch_closed", "reject": "passage_rejected"}.get(stage["kind"])
            if required:
                evidence = [e for e in events if e["stage_id"] == stage["id"] and e["event"] == required]
                result["observation_evidence"] = evidence
                result["passed"] = result["passed"] and bool(evidence)
            result["scheduled_completion"] = any(e["stage_id"] == stage["id"] and e["event"] == "stage_completed" for e in events)
            result["passed"] = result["passed"] and result["scheduled_completion"]
        checks = dict(navigation_checks)
        checks["all_stages_passed"] = all(r["passed"] for r in self.results)
        checks["no_manual_intervention"] = not any(e["event"] == "manual_intervention" for e in events)
        return {"passed": all(checks.values()), "result": status, "checks": checks,
                "stages": self.results, "manual_interventions": sum(e["event"] == "manual_intervention" for e in events),
                "recovery_count": sum(e.get("state") == "recovering" for e in events),
                "scope": "velocity_level_ordered_course_navigation", "physical_gait_validated": False}
