"""Regression coverage for learned-policy safety and lifecycle integration."""

import json
from pathlib import Path
import sys
import unittest

import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / 'controllers' / 'sensor_capture'))
from navigation_core.contracts import MotionObservation, NavigationObservation, VelocityCommand
from navigation_core.local_policy import LocalObservationEncoder, guarded_policy_command
from navigation_core.session import NavigationSession


class Policy:
    def __init__(self, action):
        self.action = action
        self.calls = 0

    def predict(self, session, now):
        self.calls += 1
        return self.action


class LocalPolicyTests(unittest.TestCase):
    def setUp(self):
        from mujoco_navigation import configure_navigation
        config = json.loads((PROJECT / 'config/sensor_lab.json').read_text())
        configure_navigation(config, 'sensor_lab')
        self.session = NavigationSession(config, [0, 0, 0])
        self.session.grid.log_odds[:] = -1
        self.preferred = VelocityCommand(0.1, 0, 1, 1.3)

    def test_encoder_shape_finite_and_unknown_edges(self):
        encoded = LocalObservationEncoder().encode(self.session, 1)
        self.assertEqual(encoded.shape, (LocalObservationEncoder.dimension,))
        self.assertTrue(np.isfinite(encoded).all())
        self.assertTrue((np.abs(encoded) <= 1).all())

    def test_guard_rejects_nan_and_wrong_shape(self):
        for action in ([np.nan, 0], [1], [1, 0, 0]):
            command = guarded_policy_command(self.session, Policy(action), self.preferred, 1)
            self.assertTrue(command.is_zero)
            self.assertTrue(self.session.local_policy_debug['intervened'])

    def test_encoder_failure_stops_before_inference(self):
        self.session.local_policy_preparation_error = 'invalid_grid'
        policy = Policy([1, 0])
        command = guarded_policy_command(self.session, policy, self.preferred, 1)
        self.assertTrue(command.is_zero)
        self.assertEqual(policy.calls, 0)

    def test_guard_rejects_occupied_swept_body(self):
        self.session.grid.log_odds[:] = 1
        command = guarded_policy_command(self.session, Policy([1, 0]), self.preferred, 1)
        self.assertTrue(command.is_zero)
        self.assertEqual(self.session.local_policy_debug['reason'], 'unsafe_occupied')

    def test_guard_clips_and_maps_actions(self):
        command = guarded_policy_command(self.session, Policy([4, 0]), self.preferred, 1)
        self.assertAlmostEqual(command.linear_x_m_s, 0.15)
        self.assertTrue(self.session.local_policy_debug['clipped'])

    def test_cancellation_and_stale_observation_precede_policy(self):
        policy = Policy([1, 0])
        self.session.local_policy = policy
        self.session.last_observation = NavigationObservation(0, 'front_lidar', [], True, 'test')
        self.session.last_valid_motion = 1
        command = self.session.tick(1, MotionObservation(1, 0, 0))
        self.assertTrue(command.is_zero)
        self.assertEqual(policy.calls, 0)
        command = self.session.tick(1, MotionObservation(1, 0, 0), cancelled=True)
        self.assertTrue(command.is_zero)
        self.assertEqual(policy.calls, 0)

    def test_policy_timeout_stops(self):
        self.session.settings['policy_timeout_s'] = -1
        command = guarded_policy_command(self.session, Policy([1, 0]), self.preferred, 1)
        self.assertTrue(command.is_zero)
        self.assertEqual(self.session.local_policy_debug['reason'], 'policy_timeout')

    def test_replay_event_float_tolerance_keeps_structure_checks(self):
        from tools.replay_navigation import events_equal
        event = {'event': 'passage_rejected', 'time': 222.20000000000002, 'count': 345}
        self.assertTrue(events_equal(event, dict(event, time=222.2)))
        self.assertFalse(events_equal(event, dict(event, time=222.21)))
        self.assertFalse(events_equal(event, dict(event, count=344)))
        self.assertFalse(events_equal([event], []))


if __name__ == '__main__':
    unittest.main()
