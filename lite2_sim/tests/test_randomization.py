"""Regression checks for reproducible randomization profiles and sampling."""

import json
from pathlib import Path
import sys
import unittest

import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "controllers" / "sensor_capture"))

from navigation_core import randomization


SIM2REAL_V1_RANGES = {
    "lidar.range_noise_m": (0.0, 0.03),
    "lidar.range_bias_m": (-0.02, 0.02),
    "lidar.dropout_probability": (0.0, 0.08),
    "lidar.beam_block_probability": (0.0, 0.05),
    "lidar.missing_scan_probability": (0.0, 0.05),
    "lidar.scan_latency_s": (0.0, 0.12),
    "odometry.linear_scale_error": (0.90, 1.10),
    "odometry.yaw_scale_error": (0.92, 1.08),
    "odometry.linear_noise_fraction": (0.01, 0.05),
    "odometry.yaw_noise_rad_s": (0.004, 0.02),
    "odometry.yaw_bias_rad_s": (-0.01, 0.01),
    "odometry.slip_probability": (0.0, 0.03),
    "odometry.slip_ratio": (0.2, 0.8),
    "imu.roll_pitch_noise_rad": (0.0, 0.01),
    "actuation.command_latency_s": (0.0, 0.15),
    "actuation.acceleration_scale": (0.7, 1.3),
    "actuation.linear_gain_error": (0.9, 1.10),
    "initial_pose.dx_m": (-0.08, 0.08),
    "initial_pose.dy_m": (-0.08, 0.08),
    "initial_pose.yaw_offset_rad": (-0.03, 0.03),
}
LIDAR_RETURN_KEYS = {"noise_m", "range_bias_m", "dropout_probability",
                     "beam_block_probability", "scan_latency_s"}


def load_raw_profile(profile_id):
    return json.loads((PROJECT / "config" / "randomization" / (profile_id + ".json"))
                      .read_text(encoding="utf-8"))


class ProfileLoadingTests(unittest.TestCase):
    def test_profile_id_and_json_path_resolve_to_the_same_profile(self):
        by_id = randomization.load_profile("sim2real_v1")
        by_path = randomization.load_profile(
            PROJECT / "config" / "randomization" / "sim2real_v1.json")
        self.assertEqual(by_id["profile_id"], "sim2real_v1")
        self.assertEqual(by_id["profile_id"], by_path["profile_id"])
        self.assertEqual(sorted(by_id["parameters"]), sorted(by_path["parameters"]))
        self.assertEqual(by_id["source_path"], "config/randomization/sim2real_v1.json")

    def test_base_dir_resolves_profile_ids_and_relative_paths(self):
        by_id = randomization.load_profile("sim2real_v1", base_dir=PROJECT)
        by_relative = randomization.load_profile(
            "config/randomization/sim2real_v1.json", base_dir=PROJECT)
        self.assertEqual(by_id["parameters"], by_relative["parameters"])

    def test_unknown_profile_reports_the_available_profiles(self):
        with self.assertRaises(FileNotFoundError) as context:
            randomization.load_profile("does_not_exist")
        self.assertIn("sim2real_v1", str(context.exception))

    def test_available_profiles_lists_the_shipped_profiles(self):
        profiles = randomization.available_profiles()
        self.assertIn("ideal", profiles)
        self.assertIn("sim2real_v1", profiles)

    def test_sim2real_v1_ranges_match_the_frozen_contract(self):
        profile = randomization.load_profile("sim2real_v1")
        self.assertEqual(set(profile["parameters"]), set(SIM2REAL_V1_RANGES))
        for flat_key, (low, high) in SIM2REAL_V1_RANGES.items():
            parameter = profile["parameters"][flat_key]
            self.assertAlmostEqual(parameter["low"], low, places=12, msg=flat_key)
            self.assertAlmostEqual(parameter["high"], high, places=12, msg=flat_key)

    def test_ideal_profile_is_the_noise_free_baseline(self):
        profile = randomization.load_profile("ideal")
        sampled = randomization.sample(profile, 3)
        for flat_key, value in sampled["values"].items():
            if flat_key.endswith("scale_error") or flat_key in (
                    "actuation.acceleration_scale", "actuation.linear_gain_error"):
                self.assertEqual(value, 1.0, flat_key)
            else:
                self.assertEqual(value, 0.0, flat_key)
        overrides = randomization.apply_to_lidar(sampled["values"], {})
        self.assertEqual(set(overrides), LIDAR_RETURN_KEYS)
        self.assertTrue(all(value == 0.0 for value in overrides.values()))

    def test_invalid_profiles_are_rejected(self):
        with self.assertRaises(ValueError):
            randomization.normalize_profile({"profile_id": "x", "lidar": {"noise": [1.0, 0.0]}})
        with self.assertRaises(ValueError):
            randomization.normalize_profile({"profile_id": "x", "lidar": {"noise": "loud"}})
        with self.assertRaises(ValueError):
            randomization.normalize_profile({"profile_id": "x", "schema_version": 2,
                                             "lidar": {"noise": [0.0, 1.0]}})
        with self.assertRaises(ValueError):
            randomization.normalize_profile({"schema_version": 1, "lidar": {"noise": 0.1}})

    def test_unknown_provenance_label_is_rejected(self):
        with self.assertRaises(ValueError):
            randomization.normalize_profile({"profile_id": "x", "provenance": {"default": "guess"},
                                             "lidar": {"noise": [0.0, 1.0]}})

    def test_null_range_falls_back_to_the_profile_baseline(self):
        profile = randomization.normalize_profile({
            "profile_id": "baseline_only",
            "baseline": {"lidar": {"range_noise_m": 0.02}},
            "lidar": {"range_noise_m": None},
        })
        parameter = profile["parameters"]["lidar.range_noise_m"]
        self.assertEqual(parameter["distribution"], "constant")
        self.assertEqual(parameter["low"], 0.02)
        sampled = randomization.sample(profile, 11)
        self.assertEqual(sampled["values"]["lidar.range_noise_m"], 0.02)


class SamplingTests(unittest.TestCase):
    def setUp(self):
        self.profile = randomization.load_profile("sim2real_v1")

    def test_same_seed_reproduces_bit_identical_values(self):
        first = randomization.sample(self.profile, 42)
        second = randomization.sample(self.profile, 42)
        self.assertEqual(first, second)
        self.assertEqual(json.dumps(first, sort_keys=True), json.dumps(second, sort_keys=True))

    def test_different_seeds_differ_while_staying_inside_the_ranges(self):
        base = randomization.sample(self.profile, 42)
        other = randomization.sample(self.profile, 43)
        self.assertNotEqual(base["values"], other["values"])
        for seed in range(42, 52):
            sampled = randomization.sample(self.profile, seed)
            self.assertEqual(sampled["seed"], seed)
            for flat_key, value in sampled["values"].items():
                parameter = self.profile["parameters"][flat_key]
                self.assertGreaterEqual(value, parameter["low"], flat_key)
                self.assertLessEqual(value, parameter["high"], flat_key)

    def test_values_and_provenance_use_the_same_flat_keys(self):
        sampled = randomization.sample(self.profile, 7)
        self.assertEqual(set(sampled), {"profile_id", "seed", "values", "provenance"})
        self.assertEqual(set(sampled["values"]), set(sampled["provenance"]))
        self.assertEqual(set(sampled["values"]), set(SIM2REAL_V1_RANGES))
        labels = set(sampled["provenance"].values())
        self.assertTrue(labels)
        self.assertTrue(labels <= set(randomization.PROVENANCE_LABELS))
        self.assertEqual(labels, set(randomization.PROVENANCE_LABELS))

    def test_sampling_leaves_the_global_numpy_random_state_untouched(self):
        np.random.seed(20261112)
        state_before = np.random.get_state()
        randomization.sample(self.profile, 5)
        state_after = np.random.get_state()
        self.assertEqual(state_before[0], state_after[0])
        np.testing.assert_array_equal(state_before[1], state_after[1])
        self.assertEqual(state_before[2:], state_after[2:])

        np.random.seed(20261112)
        expected = np.random.random(4)
        np.random.seed(20261112)
        randomization.sample(self.profile, 5)
        randomization.sample(randomization.load_profile("ideal"), 6)
        np.testing.assert_array_equal(expected, np.random.random(4))

    def test_sample_accepts_a_raw_profile_mapping(self):
        raw = load_raw_profile("sim2real_v1")
        from_raw = randomization.sample(raw, 42)
        from_loaded = randomization.sample(self.profile, 42)
        self.assertEqual(from_raw, from_loaded)

    def test_invalid_seeds_are_rejected(self):
        for seed in (-1, 1.5, "42", None, True):
            with self.assertRaises(ValueError, msg=repr(seed)):
                randomization.sample(self.profile, seed)


class ApplicationTests(unittest.TestCase):
    def test_apply_to_lidar_returns_only_the_five_contract_keys(self):
        values = {"lidar.range_noise_m": 0.01, "lidar.range_bias_m": -0.004,
                  "lidar.dropout_probability": 0.02, "lidar.beam_block_probability": 0.03,
                  "lidar.scan_latency_s": 0.04, "odometry.linear_noise_fraction": 0.02}
        settings = {"noise_m": 0.0, "frequency_hz": 10.0}
        overrides = randomization.apply_to_lidar(values, settings)
        self.assertEqual(overrides, {"noise_m": 0.01, "range_bias_m": -0.004,
                                     "dropout_probability": 0.02, "beam_block_probability": 0.03,
                                     "scan_latency_s": 0.04})
        self.assertEqual(settings, {"noise_m": 0.0, "frequency_hz": 10.0})

    def test_apply_to_lidar_omits_parameters_absent_from_values(self):
        overrides = randomization.apply_to_lidar({"lidar.missing_scan_probability": 0.02}, {})
        self.assertEqual(overrides, {})

    def test_executor_parameters_returns_the_flat_odometry_actuation_pose_values(self):
        sampled = randomization.sample("sim2real_v1", 9)
        selected = randomization.executor_parameters(sampled["values"])
        self.assertTrue(selected)
        self.assertEqual(set(selected),
                         {key for key in sampled["values"]
                          if key.split(".", 1)[0] in ("odometry", "actuation", "initial_pose")})
        for flat_key, value in selected.items():
            self.assertEqual(value, sampled["values"][flat_key])
            self.assertIsInstance(value, float)

    def test_apply_sampling_sets_the_config_block(self):
        config = {}
        sampled = randomization.apply_sampling(config, "sim2real_v1", 42)
        self.assertIs(config["randomization_sampled"], sampled)
        self.assertEqual(set(config["randomization_sampled"]),
                         {"profile_id", "seed", "values", "provenance"})


class RecordTests(unittest.TestCase):
    def test_record_is_json_serializable_and_covers_every_parameter(self):
        sampled = randomization.sample("sim2real_v1", 42)
        audit = randomization.record(sampled)
        self.assertEqual(audit["schema_version"], randomization.SCHEMA_VERSION)
        self.assertEqual(audit["profile_id"], "sim2real_v1")
        self.assertEqual(audit["seed"], 42)
        self.assertEqual(audit["parameter_count"], len(sampled["values"]))
        self.assertEqual(set(audit["values"]), set(audit["provenance"]))
        self.assertEqual(sum(audit["provenance_summary"].values()), audit["parameter_count"])
        self.assertEqual(json.loads(json.dumps(audit, allow_nan=False)), audit)

    def test_record_rejects_inconsistent_input(self):
        with self.assertRaises(ValueError):
            randomization.record({"profile_id": "x", "seed": 1, "values": {},
                                  "provenance": {"lidar.range_noise_m": "sim_assumed"}})
        with self.assertRaises(ValueError):
            randomization.record({"profile_id": "x", "seed": 1,
                                  "values": {"a.b": 1.0},
                                  "provenance": {"a.b": "not_a_label"}})
        with self.assertRaises(ValueError):
            randomization.record({"profile_id": "x"})

    def test_write_record_round_trips_and_is_byte_stable_for_one_seed(self):
        import tempfile
        with tempfile.TemporaryDirectory() as folder:
            first = Path(folder) / "randomization.json"
            second = Path(folder) / "copy.json"
            sampled = randomization.sample("sim2real_v1", 42)
            randomization.write_record(sampled, first)
            randomization.write_record(randomization.sample("sim2real_v1", 42), second)
            self.assertEqual(first.read_bytes(), second.read_bytes())
            self.assertEqual(json.loads(first.read_text(encoding="utf-8")),
                             randomization.record(sampled))


if __name__ == "__main__":
    unittest.main()