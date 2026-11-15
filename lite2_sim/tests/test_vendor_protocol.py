"""Regression checks for the actual Lite2 wire layout and SI velocity packets."""
from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "hardware"))
from vendor_protocol import decode_state, decode_command, velocity_packets, latest_mode_ack


class VendorProtocolTests(unittest.TestCase):
    def test_captured_state_offsets(self):
        packet = bytes.fromhex(
            "01090000b80000000100000001000000000000000000000039b6ca3f"
            "000000806c84ecbf000000c08b4cd33f00000040ae1955bf000000a0cc4f50bf"
            "000000001853d03e000000603849c13f00000060f9d7a23f000000a0eccd2340"
            "c88907c38076d03fb3237c7cdbe2c1bfdb2447961cbcd23f1a4c5c4301b75fbf"
            "68402983c09da33f8a66c23d2a78b73ff9eb21852d98a3bfb854d6eeeaef8a3f"
            "6bcb17ae893cb73f0f0000000000000000000000000000000000000000004d40"
            "0000000000000000")
        state = decode_state(packet)
        self.assertEqual(state["basic"], 1)
        self.assertEqual(state["motion"], 0)
        self.assertAlmostEqual(state["rpy_deg"][0], 0.208685994)
        self.assertAlmostEqual(state["position"][0], 0.257232848)
        self.assertIsNone(decode_state(packet[:-1]))

    def test_velocity_layout_matches_vendor_bridge(self):
        packets = velocity_packets(0.05, -0.1)
        self.assertEqual([len(p) for p in packets], [20, 20, 20])
        self.assertEqual([struct.unpack("<3id", p) for p in packets],
                         [(320, 8, 1, 0.05), (325, 8, 1, 0.0), (321, 8, 1, -0.1)])

    def test_remote_takeover_feedback(self):
        packet = struct.pack("<3i6d", 2309, 48, 1, 0.2, 0.0, -0.1, 0.05, 0.0, 0.0)
        result = decode_command(packet)
        self.assertEqual(result["joystick"], [0.2, 0.0, -0.1])
        self.assertEqual(result["goal_velocity"], [0.05, 0.0, 0.0])
        self.assertIsNone(decode_command(packet[:-1]))

    def test_mode_recovery_uses_received_packets(self):
        self.assertIsNone(latest_mode_ack("AorM = 4\n"))
        self.assertEqual(latest_mode_ack("(553716739, 0, 0)\n"), 0x21010C03)
        self.assertEqual(latest_mode_ack("(553716739, 0, 0)\n(553716738, 0, 0)"), 0x21010C02)


if __name__ == "__main__":
    unittest.main()
