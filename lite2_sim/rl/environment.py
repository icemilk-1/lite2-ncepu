"""Gymnasium local navigation using the project's observed navigation stack."""

import copy
import csv
import json
import math
from pathlib import Path
import sys
import tempfile
import time

import gymnasium as gym
import mujoco
import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / 'controllers' / 'sensor_capture'))

from mujoco_navigation import MujocoVelocityExecutor, configure_navigation
from navigation_core.contracts import MotionObservation
from navigation_core.control import NavigationController
from navigation_core.local_policy import LocalObservationEncoder
from navigation_core.mujoco_adapter import MujocoLidarObservationAdapter
from navigation_core.session import NavigationSession
from navigation_core.terrain import TerrainGrid, project_cloud
from tools.prepare_mujoco import build_scene


class ActionPolicy:
    def __init__(self):
        self.action = np.array([-1.0, 0.0], dtype=np.float32)

    def predict(self, session, now):
        return self.action


class LocalNavigationEnv(gym.Env):
    metadata = {'render_modes': []}

    def __init__(self, task='tracking', baseline=False, max_seconds=None):
        if task not in ('tracking', 'obstacle', 'dynamic', 'obstacle_bypass'):
            raise ValueError('Unknown RL task: ' + task)
        self.task, self.baseline = task, baseline
        default_seconds = 150 if task == 'obstacle_bypass' else 35
        self.max_seconds = float(default_seconds if max_seconds is None else max_seconds)
        if not math.isfinite(self.max_seconds) or self.max_seconds <= 0:
            raise ValueError('Episode duration must be positive and finite.')
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(-1.0, 1.0,
                                              shape=(LocalObservationEncoder.dimension,), dtype=np.float32)
        self.directory = tempfile.TemporaryDirectory(prefix='lite2_rl_')
        scene = 'ruins_reference' if task == 'obstacle_bypass' else 'sensor_lab'
        self.config = json.loads((PROJECT / 'config' / (scene + '.json')).read_text(encoding='utf-8'))
        configure_navigation(self.config, scene)
        if scene == 'sensor_lab':
            self.config['calibration_board']['position_m'] = [30, 30, 0.55]
            self.config['obstacles'] = []
            if task != 'tracking':
                self.config['obstacles'] = [dict(name='rl_obstacle', position_m=[-3, -3, 0.25],
                                                size_m=[0.3, 0.3, 0.5])]
        self.model, self.data, _ = build_scene(self.config, scene, Path(self.directory.name))
        self.original_geom_pos = self.model.geom_pos.copy()
        self.episode_done = True

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        started = time.perf_counter()
        config = copy.deepcopy(self.config)
        config['navigation']['random_seed'] = int(self.np_random.integers(0, 2**30))
        config['navigation']['navigation_timeout_s'] = self.max_seconds
        config['navigation']['policy_timeout_s'] = 0.05
        config['navigation']['policy_fallback'] = False
        self.model.geom_pos[:] = self.original_geom_pos
        mujoco.mj_resetData(self.model, self.data)
        if self.task == 'obstacle_bypass':
            stage = next(s for s in config['course']['stages'] if s['id'] == 'obstacle_bypass')
            start = list(stage['start_xy_m'])
            goal = list(stage['checkpoints'][-1]['xy_m'])
        else:
            start = [-4.0, -3.0 + float(self.np_random.uniform(-0.15, 0.15))]
            distance = 0.6 if self.task == 'tracking' else 2.0
            goal = [start[0] + distance, start[1] + float(self.np_random.uniform(-0.15, 0.15))]
        config['robot_position_m'][:2] = start
        config['navigation']['goal_xy_m'] = goal
        self.episode_config = config
        self.executor = MujocoVelocityExecutor(self.model, self.data, config)
        self.executor.pose = [*start, float(self.np_random.uniform(-0.15, 0.15))]
        self.executor.set_pose(self.data, self.executor.pose, 0.0, 0.0, 0.0)
        for name, target in config['joint_positions_rad'].items():
            joint = self.model.joint(name)
            self.data.qpos[joint.qposadr[0]] = target
            self.data.ctrl[self.model.actuator(name + '_servo').id] = target
        mujoco.mj_forward(self.model, self.data)
        grid = TerrainGrid(config['field_size_m'], config['navigation']['grid_resolution_m'], config['navigation'])
        self.session = NavigationSession(config, self.executor.initial_estimate_pose(), grid)
        self.action_policy = ActionPolicy()
        if not self.baseline:
            self.session.local_policy = self.action_policy
        self.encoder = LocalObservationEncoder()
        self.lidar = MujocoLidarObservationAdapter(self.model, self.data, config['lidar'],
                                                  self.executor.base_height + config['lidar']['mount_xyz_m'][2])
        self.now = self.next_scan = 0.0
        self.motion = MotionObservation(0, 0, 0)
        self.session.motion = self.motion
        self.steps = self.interventions = 0
        self.last_action = np.array([-1.0, 0.0], dtype=np.float32)
        self.timings = {'sensor_s': 0.0, 'control_s': 0.0, 'executor_s': 0.0, 'encoding_s': 0.0}
        self.control_times = []
        self.cycle_times = []
        self.record_directory = None
        self.recorded_trace = []
        self.minimum_clearance_m = 1.0
        self.action_variation = 0.0
        self.reward_totals = {}
        self.obstacle_center = [-3.0, -3.0 + float(self.np_random.uniform(-0.25, 0.25)), 0.25]
        if self.task in ('obstacle', 'dynamic'):
            self.model.geom_pos[self.model.geom('rl_obstacle').id] = self.obstacle_center
            config['obstacles'][0]['position_m'] = list(self.obstacle_center)
            mujoco.mj_forward(self.model, self.data)
        config['rl_episode'] = dict(task=self.task, seed=seed, initial_yaw_rad=self.executor.pose[2],
                                    max_seconds=self.max_seconds, obstacle_center=self.obstacle_center,
                                    dynamic_amplitude_m=0.65, dynamic_angular_frequency_rad_s=0.25)
        self._observe()
        self.episode_done = False
        self.reset_duration_s = time.perf_counter() - started
        return self._encode(), {'reset_duration_s': self.reset_duration_s}

    def _observe(self):
        if self.now + 1e-8 < self.next_scan:
            return
        started = time.perf_counter()
        if self.task == 'dynamic':
            self.model.geom_pos[self.model.geom('rl_obstacle').id, 1] = self.obstacle_center[1] + 0.65 * math.sin(self.now * 0.25)
            mujoco.mj_forward(self.model, self.data)
        self.lidar.read(self.now)
        observation = project_cloud(self.lidar.raw, self.config['lidar'], self.motion,
                                    self.executor.base_height + self.motion.ground_height_m)
        self.session.observe(observation)
        self.next_scan += 1.0 / self.config['lidar']['frequency_hz']
        self.timings['sensor_s'] += time.perf_counter() - started

    def _encode(self):
        started = time.perf_counter()
        result = self.encoder.encode(self.session, self.now)
        self.timings['encoding_s'] += time.perf_counter() - started
        return result

    def step(self, action):
        cycle_started = time.perf_counter()
        if self.episode_done:
            raise RuntimeError('Call reset before stepping a finished episode.')
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (2,) or not np.isfinite(action).all():
            raise ValueError('Action must contain two finite values.')
        action = np.clip(action, -1.0, 1.0)
        self.action_policy.action = action.copy()
        self.session.local_policy_debug = None
        started = time.perf_counter()
        command = self.session.tick(self.now, self.motion)
        duration = time.perf_counter() - started
        self.timings['control_s'] += duration
        self.control_times.append(duration)
        debug = self.session.local_policy_debug or {}
        intervened = bool(debug.get('intervened'))
        self.interventions += int(intervened)
        path = tuple(self.session.plan.path)
        before_arc = NavigationController._path_arc(self.executor.pose, path) if path else 0.0
        dt = self.config['navigation']['control_period_s']
        started = time.perf_counter()
        self.motion, collision = self.executor.step(command, self.now, dt)
        self.timings['executor_s'] += time.perf_counter() - started
        self.session.propagate(self.motion, dt)
        self.session.motion = self.motion
        self.now = round(self.now + dt, 8)
        self.steps += 1
        probe_center = self.data.geom_xpos[self.executor.probe]
        candidates = self.executor.environment[
            np.linalg.norm(self.data.geom_xpos[self.executor.environment] - probe_center, axis=1)
            < self.model.geom_rbound[self.executor.environment] + 1.4]
        for geometry in candidates:
            if self.model.geom(int(geometry)).name != 'floor':
                distance = float(mujoco.mj_geomDistance(self.model, self.data, self.executor.probe,
                                                      int(geometry), 1.0, None))
                self.minimum_clearance_m = min(self.minimum_clearance_m, distance)
        self._observe()
        after_arc = NavigationController._path_arc(self.executor.pose, path) if path else before_arc
        progress = float(np.clip(after_arc - before_arc, -0.01, 0.01))
        goal_distance = math.dist(self.executor.pose[:2], self.session.goal)
        success = (self.session.status == 'goal_reached' and goal_distance <= 0.25
                   and self.session.stopped_since is not None
                   and self.now - self.session.stopped_since >= 1.0)
        terminal_failure = self.session.finished and self.session.status != 'goal_reached'
        terminated = bool(collision or success or (terminal_failure and self.session.status != 'timeout'))
        truncated = bool(not terminated and (self.now >= self.max_seconds or self.session.status == 'timeout'))
        # A stationary goal wait has no continuing time/action penalty.
        waiting = self.session.status == 'goal_reached'
        terms = dict(progress=10.0 * progress, success=10.0 * float(success), collision=-10.0 * float(collision),
                     time=0.0 if waiting else -0.002,
                     smoothness=0.0 if waiting else -0.002 * float(np.mean(np.square(action - self.last_action))),
                     intervention=-0.02 * float(intervened))
        for name, value in terms.items():
            self.reward_totals[name] = self.reward_totals.get(name, 0.0) + value
        used_action = action
        if self.baseline:
            used_action = np.array([2 * command.linear_x_m_s / self.session.settings['max_linear_speed_m_s'] - 1,
                                    command.angular_z_rad_s / self.session.settings['max_yaw_speed_rad_s']])
        if self.baseline or debug.get('raw_action') is not None:
            self.action_variation += float(np.mean(np.abs(used_action - self.last_action)))
            self.last_action = used_action.copy()
            self.encoder.previous_action = used_action.copy()
        self.episode_done = terminated or truncated
        info = dict(success=success, collision=bool(collision), intervention=intervened,
                    status=self.session.status, reward_terms=terms, elapsed_s=self.now,
                    goal_error_m=goal_distance, distance_m=self.executor.distance_m,
                    applied_action=[command.linear_x_m_s, command.angular_z_rad_s],
                    executed_action=self.executor.velocity.tolist(), policy_debug=debug)
        observation = self._encode()
        self.cycle_times.append(time.perf_counter() - cycle_started)
        if self.record_directory is not None:
            row = [self.now, *self.executor.pose, self.executor.ground,
                   *action, command.linear_x_m_s, command.angular_z_rad_s, *self.executor.velocity]
            if self.task in ('obstacle', 'dynamic'):
                row.extend(self.model.geom_pos[self.model.geom('rl_obstacle').id].tolist())
            self.recorded_trace.append(row)
        if self.episode_done:
            info['episode_metrics'] = dict(success=success, elapsed_s=self.now, goal_error_m=goal_distance,
                                           collisions=self.executor.collisions, interventions=self.interventions,
                                           distance_m=self.executor.distance_m, steps=self.steps,
                                           control_p95_s=float(np.percentile(self.control_times, 95)),
                                           control_max_s=max(self.control_times), timing=self.timings.copy(),
                                           minimum_clearance_m=self.minimum_clearance_m,
                                           clearance_capped_at_m=1.0,
                                           action_variation_mean=self.action_variation / self.steps,
                                           terminal_status=self.session.status,
                                           reward_totals=self.reward_totals.copy(),
                                           cycle_p95_s=float(np.percentile(self.cycle_times, 95)),
                                           cycle_max_s=max(self.cycle_times))
            if self.record_directory is not None:
                with (self.record_directory / 'slam_trajectory.csv').open('w', newline='', encoding='utf-8') as stream:
                    writer = csv.writer(stream)
                    header = ['sim_time_s', 'truth_x_m_eval', 'truth_y_m_eval', 'truth_yaw_rad_eval',
                              'ground_height_m_eval', 'raw_action_0', 'raw_action_1',
                              'command_v', 'command_w', 'executed_v_m_s', 'executed_w_rad_s']
                    if self.task in ('obstacle', 'dynamic'):
                        header.extend(['obstacle_x_m', 'obstacle_y_m', 'obstacle_z_m'])
                    writer.writerow(header)
                    writer.writerows(self.recorded_trace)
                (self.record_directory / 'episode_metrics.json').write_text(
                    json.dumps(info['episode_metrics'], indent=2), encoding='utf-8')
        return observation, sum(terms.values()), terminated, truncated, info

    def start_recording(self, directory):
        self.record_directory = Path(directory)
        self.record_directory.mkdir(parents=True, exist_ok=True)
        mujoco.mj_saveLastXML(str(self.record_directory / 'scene.xml'), self.model)
        (self.record_directory / 'config_snapshot.json').write_text(
            json.dumps(self.episode_config, indent=2), encoding='utf-8')

    def close(self):
        self.directory.cleanup()
