"""Hardware command freshness, limiting and takeover regression checks."""
from pathlib import Path
import ast
import struct
import sys
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "hardware"))
from execution_guard import fresh, guarded_velocity
from vendor_protocol import velocity_packets


class ExecutionGuardTests(unittest.TestCase):
    def test_duplicate_bridge_does_not_register_ros_node(self):
        source = Path(__file__).resolve().parents[1] / "hardware/lite2_live.py"
        tree = ast.parse(source.read_text(encoding="utf8"))
        function = next(item for item in tree.body if isinstance(item, ast.FunctionDef)
                        and item.name == "bridge")
        socket_module = Mock()
        receiver = socket_module.socket.return_value
        receiver.bind.side_effect = OSError("address already in use")
        ros = Mock()
        scope = {"socket": socket_module, "rospy": ros}
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), "exec"), scope)
        with self.assertRaises(OSError):
            scope["bridge"]()
        ros.init_node.assert_not_called()
        receiver.close.assert_called_once()

    def setUp(self):
        self.state = dict(received=10.0, command_received=10.0, basic=6,
                          joystick=[0.0, 0.0, 0.0], rpy_deg=[0.0, 0.0, 0.0], error_state=0)

    def command(self, now=10.1, created=10.0, depth_time=10.0, armed=True, mode=True,
                linear=0.02, angular=0.0):
        return guarded_velocity([linear, angular, created], now, self.state,
                                armed, mode, depth_time)

    def test_valid_command_and_limits(self):
        self.assertEqual(self.command(), (0.02, 0.0, "enabled"))
        self.assertEqual(self.command(linear=2, angular=-2), (1.0, -1.0, "enabled"))
        self.assertEqual(self.command(linear=-0.1), (-0.1, 0.0, "enabled"))

    def test_reverse_command_preserves_sign_and_limits(self):
        self.assertEqual(self.command(linear=-0.02), (-0.02, 0.0, "enabled"))
        self.assertEqual(self.command(linear=-2.0), (-1.0, 0.0, "enabled"))
        self.assertEqual(self.command(linear=0.0), (0.0, 0.0, "enabled"))

    def test_configured_limits_and_existing_gates(self):
        limits = dict(max_forward_m_s=0.1, max_reverse_m_s=0.08, max_yaw_rad_s=0.2)
        self.assertEqual(guarded_velocity([0.12, -0.3, 10], 10.1, self.state,
                                         True, True, 10, limits), (0.1, -0.2, "enabled"))
        self.assertEqual(guarded_velocity([-0.12, 0, 10], 10.1, self.state,
                                         True, True, 10, limits), (-0.08, 0, "enabled"))
        self.assertEqual(guarded_velocity([0.1, 0, 10], 10.1, self.state,
                                         False, True, 10, limits), (0, 0, "disarmed"))

    def test_invalid_configured_limits_stop(self):
        for value in (-1, float("nan"), float("inf"), "invalid", None):
            with self.subTest(value=value):
                self.assertEqual(guarded_velocity([0.02, 0, 10], 10.1, self.state,
                                                 True, True, 10,
                                                 {"max_forward_m_s": value}),
                                 (0, 0, "invalid_limits"))

    def test_reverse_command_retains_execution_gates(self):
        self.assertEqual(self.command(linear=-0.02, armed=False),
                         (0.0, 0.0, "disarmed"))
        self.assertEqual(self.command(linear=-0.02, mode=False),
                         (0.0, 0.0, "mode_unconfirmed"))
        self.assertEqual(self.command(linear=-0.02, created=9.8),
                         (0.0, 0.0, "command_timeout"))
        self.assertEqual(self.command(linear=-0.02, depth_time=9.0),
                         (0.0, 0.0, "stale_depth"))

    def test_reverse_command_reaches_vendor_packet_as_signed_double(self):
        linear, angular, reason = self.command(linear=-0.02)
        self.assertEqual(reason, "enabled")
        packets = velocity_packets(linear, angular)
        self.assertEqual(struct.unpack("<3id", packets[0]), (320, 8, 1, -0.02))

    def test_disarmed_and_unconfirmed_modes(self):
        self.assertEqual(self.command(armed=False)[2], "disarmed")
        self.assertEqual(self.command(mode=False)[2], "mode_unconfirmed")
        self.state["basic"] = 1
        self.assertEqual(self.command()[2], "motion_mode_not_ready")

    def test_source_state_and_feedback_expiry(self):
        self.state["received"] = 9.0
        self.assertEqual(self.command()[2], "stale_state")
        self.state["received"] = 10.0
        self.state["command_received"] = 9.0
        self.assertEqual(self.command()[2], "stale_command_feedback")

    def test_process_exit_command_timeout(self):
        self.assertEqual(self.command(created=9.8), (0.0, 0.0, "command_timeout"))

    def test_old_depth_retains_original_timestamp(self):
        self.assertEqual(self.command(depth_time=9.0), (0.0, 0.0, "stale_depth"))

    def test_future_and_nonfinite_timestamps(self):
        self.assertEqual(self.command(created=11.0)[2], "command_timeout")
        self.assertEqual(self.command(depth_time=11.0)[2], "stale_depth")
        self.assertFalse(fresh(float("nan"), 10, 0.5))
        self.assertFalse(fresh(float("inf"), 10, 0.5))

    def test_remote_takeover(self):
        self.state["joystick"][0] = 0.1
        self.assertEqual(self.command(), (0.0, 0.0, "remote_takeover"))

    def test_vendor_error_and_tilt(self):
        self.state["error_state"] = 1
        self.assertEqual(self.command()[2], "vendor_error")
        self.state["error_state"] = 0
        self.state["rpy_deg"][0] = 20
        self.assertEqual(self.command()[2], "body_tilt")

    def test_nonfinite_speed(self):
        self.assertEqual(self.command(linear=float("nan")), (0.0, 0.0, "invalid_command"))


if __name__ == "__main__":
    unittest.main()
