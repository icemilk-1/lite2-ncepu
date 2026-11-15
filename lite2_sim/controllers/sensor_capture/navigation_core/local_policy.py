"""Deployable observation encoding and guarded learned local velocity selection."""

import math
import time

import numpy as np

from navigation_core.contracts import VelocityCommand


class LocalObservationEncoder(object):
    size = 30
    beams = 72
    dimension = 3 * size * size + 4 * beams + 9

    def __init__(self):
        axis = (np.arange(self.size) + 0.5 - self.size / 2.0) * 0.1
        xx, yy = np.meshgrid(axis, axis)
        self.local_cells = np.column_stack((xx.ravel(), yy.ravel()))
        self.previous_action = np.zeros(2, dtype=np.float32)
        self.scan_time = None
        self.current_scan = np.zeros(2 * self.beams, dtype=np.float32)
        self.previous_scan = self.current_scan.copy()

    def encode(self, session, now):
        grid = session.grid
        x, y, yaw = session.pose
        c, s = math.cos(yaw), math.sin(yaw)
        channels = np.zeros((3, self.size * self.size), dtype=np.float32)
        # Four sub-samples preserve occupied/unknown/hazard cells when downsampling.
        for dx, dy in ((-0.025, -0.025), (-0.025, 0.025), (0.025, -0.025), (0.025, 0.025)):
            local_x = self.local_cells[:, 0] + dx
            local_y = self.local_cells[:, 1] + dy
            col = np.floor((x + c * local_x - s * local_y - grid.x_min) / grid.resolution).astype(int)
            row = np.floor((y + s * local_x + c * local_y - grid.y_min) / grid.resolution).astype(int)
            inside = (col >= 0) & (col < grid.width) & (row >= 0) & (row < grid.height)
            safe_col, safe_row = np.clip(col, 0, grid.width - 1), np.clip(row, 0, grid.height - 1)
            values = grid.log_odds[safe_row, safe_col]
            channels[0] = np.maximum(channels[0], inside & (values >= session.settings['occupied_log_odds_threshold']))
            channels[1] = np.maximum(channels[1], ~inside | (values > -0.18))
            hazards = getattr(grid, 'terrain_hazard', np.zeros_like(grid.log_odds, dtype=bool))
            edges = getattr(grid, 'height_discontinuities', np.zeros_like(hazards))
            channels[2] = np.maximum(channels[2], ~inside | hazards[safe_row, safe_col] | edges[safe_row, safe_col])
        observation = session.last_observation
        if observation is not None and observation.timestamp_s != self.scan_time:
            self.previous_scan = self.current_scan.copy()
            ranges = np.ones(self.beams, dtype=np.float32)
            valid = np.zeros(self.beams, dtype=np.float32)
            maximum = float(session.lidar['max_range_m'])
            mount_x, mount_y = session.lidar['mount_xy_m']
            for px, py, hit in observation.points:
                px, py = px + mount_x, py + mount_y
                sector = int((math.atan2(py, px) + math.pi) * self.beams / (2 * math.pi)) % self.beams
                valid[sector] = 1.0
                if hit:
                    ranges[sector] = min(ranges[sector], math.hypot(px, py) / maximum)
            self.current_scan = np.concatenate((ranges, valid))
            self.scan_time = observation.timestamp_s
        target = session.goal or session.pose[:2]
        if session.plan.path:
            target = session.navigator._lookahead_target(session.pose, session.plan.path, 0.45)
        dx, dy = target[0] - x, target[1] - y
        local_dx, local_dy = c * dx + s * dy, -s * dx + c * dy
        bearing = math.atan2(local_dy, local_dx)
        motion = session.motion
        maximum = session.settings['max_linear_speed_m_s']
        angular = session.settings['max_yaw_speed_rad_s']
        age = 1.0 if observation is None else min(1.0, max(0.0, now - observation.timestamp_s))
        state = [np.clip(local_dx / 1.5, -1, 1), np.clip(local_dy / 1.5, -1, 1),
                 math.sin(bearing), math.cos(bearing),
                 0.0 if motion is None else motion.linear_x_m_s / maximum,
                 0.0 if motion is None else motion.angular_z_rad_s / angular,
                 self.previous_action[0], self.previous_action[1], age]
        return np.clip(np.concatenate((channels.ravel(), self.previous_scan, self.current_scan, state)),
                       -1.0, 1.0).astype(np.float32)


def guarded_policy_command(session, policy, preferred, now):
    started = time.perf_counter()
    debug = {'raw_action': None, 'reason': None, 'intervened': False, 'fallback': False}
    linear, angular = 0.0, 0.0
    try:
        preparation_error = getattr(session, 'local_policy_preparation_error', None)
        if preparation_error is not None:
            raise ValueError('observation_encoding_' + preparation_error)
        action = np.asarray(policy.predict(session, now), dtype=float)
        if action.shape != (2,) or not np.isfinite(action).all():
            raise ValueError('invalid_action')
        debug['raw_action'] = action.tolist()
        clipped = np.clip(action, -1.0, 1.0)
        linear = (clipped[0] + 1.0) * 0.5 * session.settings['max_linear_speed_m_s']
        angular = clipped[1] * session.settings['max_yaw_speed_rad_s']
        debug['clipped'] = bool(np.any(action != clipped))
        if time.perf_counter() - started > session.settings.get('policy_timeout_s', 0.05):
            raise ValueError('policy_timeout')
        safe, _, result = session.check_local_candidate(linear, angular)
        if not safe:
            raise ValueError('unsafe_' + result)
    except Exception as error:
        linear, angular = 0.0, 0.0
        debug['reason'] = str(error) or type(error).__name__
        debug['intervened'] = True
        if session.settings.get('policy_fallback', False):
            command = session._local_rollout(preferred, now)
            linear, angular = command.linear_x_m_s, command.angular_z_rad_s
            debug['fallback'] = True
    debug['duration_s'] = time.perf_counter() - started
    debug['applied'] = [linear, angular]
    session.local_policy_debug = debug
    return VelocityCommand(linear, angular, now, now + session.settings['command_timeout_s'], source='local_policy')


class SavedLocalPolicy(object):
    def __init__(self, path):
        import torch
        from stable_baselines3 import PPO
        torch.set_num_threads(1)
        self.model = PPO.load(str(path), device='cpu')
        self.encoder = LocalObservationEncoder()
        self.observation = None
        if self.model.observation_space.shape != (self.encoder.dimension,):
            raise ValueError('Policy observation dimension differs from the deployed encoder.')
        self.model.predict(np.zeros(self.encoder.dimension, dtype=np.float32), deterministic=True)

    def prepare(self, session, now):
        self.observation = self.encoder.encode(session, now)

    def predict(self, session, now):
        action, _ = self.model.predict(self.observation, deterministic=True)
        self.encoder.previous_action = np.asarray(action, dtype=np.float32).copy()
        return action
