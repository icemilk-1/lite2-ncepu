"""Regression checks for the MuJoCo model, shared geometry and ideal LiDAR."""

import hashlib
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
sys.path.insert(0, str(PROJECT / "controllers" / "sensor_capture"))

from navigation_core.mujoco_adapter import MujocoLidarObservationAdapter
from tools import prepare_model, prepare_mujoco
from tools.scene_geometry import build_geometry


class MujocoBaselineTests(unittest.TestCase):
    def test_rubble_stays_outside_ramp_surfaces(self):
        config = json.loads((PROJECT / "config/ruins_reference.json").read_text())
        for shape in build_geometry(config, "ruins_reference"):
            if "rubble_stone_" not in shape["name"]:
                continue
            yaw = shape["rotation"][3]
            rotation = np.array([[np.cos(yaw), -np.sin(yaw)],
                                 [np.sin(yaw), np.cos(yaw)]])
            vertices = np.asarray(shape["vertices_m"])[:, :2] @ rotation.T
            vertices += np.asarray(shape["position_m"])[:2]
            lower, upper = vertices.min(axis=0), vertices.max(axis=0)
            for section in config["upper_left_ramp"]["sections"]:
                x0, x1, y0, y1 = section["bounds_m"]
                overlaps = upper[0] > x0 and lower[0] < x1 and upper[1] > y0 and lower[1] < y1
                self.assertFalse(overlaps, (shape["name"], section["name"]))

    def test_partition_culvert_and_ramp_side_opening(self):
        config = json.loads((PROJECT / "config/ruins_reference.json").read_text())
        with tempfile.TemporaryDirectory() as output:
            model, data, _ = prepare_mujoco.build_scene(config, "ruins_reference", output)
            groups = np.array([0, 0, 0, 1, 0, 0], dtype=np.uint8)

            def ray(origin):
                geometry_id = np.array([-1], dtype=np.int32)
                distance = mujoco.mj_ray(model, data, np.asarray(origin, dtype=float),
                                          np.array([1.0, 0, 0]), groups, True, -1, geometry_id)
                return distance

            gate = config["partition_culvert"]
            x, y = gate["center_m"]
            for offset in [-gate["clear_width_m"]*.3, 0, gate["clear_width_m"]*.3]:
                distance = ray([x - gate["length_m"] / 2 - 0.1, y + offset, 0.25])
                self.assertTrue(distance == -1 or distance > gate["length_m"] + 0.1)
            lower, upper = config["ramp_side_opening"]["y_limits_m"]
            for y in np.linspace(lower + 0.15, upper - 0.15, 3):
                self.assertGreater(ray([-4.8, y, 0.45]), 0.52)
            self.assertLess(ray([-4.8, lower - 0.2, 0.45]), 0.52)
            rubble_bounds = []
            for index in range(model.ngeom):
                if model.geom(index).name.startswith("inner_bay_rubble_stone_"):
                    mesh_id = model.geom_dataid[index]
                    address, count = model.mesh_vertadr[mesh_id], model.mesh_vertnum[mesh_id]
                    vertices = model.mesh_vert[address:address + count]
                    world = vertices @ data.geom_xmat[index].reshape(3, 3).T + data.geom_xpos[index]
                    rubble_bounds.append(float(world[:, 0].min()))
            self.assertAlmostEqual(min(rubble_bounds), config["ramp_side_opening"]["rubble_west_edge_m"], places=4)

    def test_confirmed_ramp_and_covered_culvert_geometry(self):
        config = json.loads((PROJECT / "config/ruins_reference.json").read_text())
        with tempfile.TemporaryDirectory() as output:
            model, data, _ = prepare_mujoco.build_scene(config, "ruins_reference", output)
            groups = np.array([0, 0, 0, 1, 0, 0], dtype=np.uint8)

            def distance(origin, direction):
                geometry_id = np.array([-1], dtype=np.int32)
                return mujoco.mj_ray(model, data, np.array(origin, dtype=float),
                                     np.array(direction, dtype=float), groups, True, -1, geometry_id)

            frame = config["frame"]
            x, y = frame["center_m"]
            entrance_y = y - frame["width_m"] / 2 - 0.15
            for offset in [-0.35, 0, 0.35]:
                for height in [frame["height_m"]*.1, frame["height_m"]*.5, frame["height_m"]*.9]:
                    with self.subTest(offset=offset, height=height):
                        hit_distance = distance([x + offset, entrance_y, height], [0, 1, 0])
                        self.assertTrue(hit_distance == -1.0 or hit_distance > frame["width_m"] + 0.15)
            self.assertAlmostEqual(distance([x, y, 0.10], [0, 0, 1]), frame["height_m"] - 0.10, places=5)
            for section in config["upper_left_ramp"]["sections"]:
                x0, x1, y0, y1 = section["bounds_m"]
                expected_height = sum(section["corner_heights_m"]) / 4
                center = np.mean(section.get("corners_xy_m", [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]), axis=0)
                self.assertAlmostEqual(2 - distance([*center, 2], [0, 0, -1]),
                                       expected_height, places=5)

            # Check near both ramp edges as well as the center. A wall crossing
            # the descending ramp previously passed the center-only check.
            for index in range(model.ngeom):
                if "rubble_stone_" in model.geom(index).name:
                    model.geom_group[index] = 5
            approach, _, descent = config["upper_left_ramp"]["sections"]
            for section, axis in [(approach, 1), (descent, 0)]:
                x0, x1, y0, y1 = section["bounds_m"]
                for along in np.linspace(0.05, 0.95, 9):
                    for across in [0.03, 0.5, 0.97]:
                        u, v = (across, along) if axis == 1 else (along, across)
                        position = [x0 + u * (x1 - x0), y0 + v * (y1 - y0), 2]
                        expected = config["upper_left_ramp"]["rise_m"] * (along if axis == 1 else 1 - along)
                        self.assertAlmostEqual(2 - distance(position, [0, 0, -1]), expected, places=5,
                                               msg=str(position))
            for x, y in [[-6, 3], [-4, 5], [-4.62, 3.2], [-3, 3.41]]:
                self.assertAlmostEqual(2 - distance([x, y, 2], [0, 0, -1]), 1.05, places=5)

    def test_shared_geometry_matches_pre_migration_webots_scenes(self):
        # Full generated scenes normalized for whitespace and temporary root.
        # Baselines captured from the original builder before geometry extraction.
        expected = {
            "sensor_lab": "66af5b2f0ff8ae2c35ba786f6bd02775c1c5879ca28b085dc411ef795aaff979",
            "ruins_reference": "1573299c64fbedea81e5a35e44d6fb3cb846ec87554e4c11fca57fc5bd17306f",
        }
        with tempfile.TemporaryDirectory() as folder:
            project = Path(folder)
            (project / "controllers/sensor_capture").mkdir(parents=True)
            with patch.object(prepare_model, "PROJECT", project):
                for scene, digest in expected.items():
                    with self.subTest(scene=scene):
                        config_path = PROJECT / "config" / (scene + ".json")
                        if scene == "ruins_reference":
                            config_path = PROJECT / "tests/fixtures/ruins_reference_v1.json"
                        config = json.loads(config_path.read_text())
                        builder = prepare_model.build_world if scene == "sensor_lab" else prepare_model.build_ruins_world
                        text = builder(config).read_text().replace(project.as_posix(), "/PROJECT")
                        normalized = "".join(text.split())
                        self.assertEqual(hashlib.sha256(normalized.encode()).hexdigest(), digest)

    def test_model_cache_reuse_source_invalidation_and_repair(self):
        with tempfile.TemporaryDirectory() as folder:
            project = Path(folder)
            shutil.copytree(PROJECT / "assets", project / "assets")
            source = project / "assets/JueyingMiniLiteV2/urdf/MiniLiteV2_Rsm.urdf"
            with patch.object(prepare_mujoco, "PROJECT", project), patch.object(
                    prepare_mujoco, "extract_model", return_value=source):
                cache, manifest = prepare_mujoco.prepare_robot()
                timestamp = (cache / "lite2.xml").stat().st_mtime_ns
                reused, _ = prepare_mujoco.prepare_robot()
                self.assertEqual(cache, reused)
                self.assertEqual(timestamp, (cache / "lite2.xml").stat().st_mtime_ns)
                self.assertEqual(len(manifest["joints"]), 12)
                self.assertEqual(len(manifest["meshes"]), 8)
                self.assertAlmostEqual(manifest["source_model_mass_kg"], 10.03)
                self.assertTrue(all(item["conversion_bounds_error_m"] < 1e-5
                                    for item in manifest["meshes"].values()))
                (cache / "model_manifest.json").write_text("invalid interrupted manifest")
                repaired, _ = prepare_mujoco.prepare_robot()
                self.assertEqual(cache, repaired)
                source.write_text(source.read_text() + "\n<!-- changed source -->\n")
                changed, changed_manifest = prepare_mujoco.prepare_robot()
                self.assertNotEqual(changed, cache)
                self.assertNotEqual(manifest["source_digest"], changed_manifest["source_digest"])

    def test_lidar_distinguishes_invalid_hit_no_hit_and_beyond_range(self):
        xml = '''<mujoco><worldbody>
          <site name="front_lidar"/>
          <geom name="robot" type="sphere" pos="0.3 0 0" size="0.05" group="0"/>
          <geom name="too_near" type="box" pos="-0.04 0 0" size="0.01 0.01 0.1" group="3"/>
          <geom name="wall" type="box" pos="2 0 0" size="0.1 0.1 0.1" group="3"/>
          <geom name="far_wall" type="box" pos="0 10 0" size="0.1 0.1 0.1" group="3"/>
        </worldbody></mujoco>'''
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        settings = {
            "model_status": "test_geometry", "number_of_layers": 1,
            "horizontal_resolution": 4, "field_of_view_rad": 2 * np.pi,
            "vertical_field_of_view_rad": 0, "min_range_m": 0.08, "max_range_m": 3,
            "obstacle_height_min_m": -0.5, "obstacle_height_max_m": 0.5, "frequency_hz": 10,
        }
        adapter = MujocoLidarObservationAdapter(model, data, settings, 0)
        observation = adapter.read(1.25)
        np.testing.assert_array_equal(adapter.raw["status"], [[0, 2, 1, 3]])
        self.assertAlmostEqual(adapter.raw["range_image_m"][0, 2], 1.9, places=5)
        self.assertTrue(np.isnan(adapter.raw["range_image_m"][0, 0]))
        self.assertEqual(observation.frame_id, "front_lidar")
        self.assertEqual(observation.timestamp_s, 1.25)
        self.assertTrue(observation.valid)

    def test_camera_axes_and_joint_limit_validation(self):
        config = json.loads((PROJECT / "config/sensor_lab.json").read_text())
        with tempfile.TemporaryDirectory() as output:
            model, data, _ = prepare_mujoco.build_scene(config, "sensor_lab", output)
            self.assertEqual(model.nu, 12)
            self.assertEqual(model.nq, 19)
            world_camera = data.cam_xmat[model.camera("front_rgb").id].reshape(3, 3)
            body_rotation = data.body("TORSO").xmat.reshape(3, 3)
            optical = body_rotation.T @ world_camera @ np.diag([1, -1, -1])
            np.testing.assert_allclose(optical, [[0, 0, 1], [-1, 0, 0], [0, -1, 0]], atol=1e-10)
            config["joint_positions_rad"]["FL_Knee"] = 100
            with self.assertRaisesRegex(ValueError, "URDF limit"):
                prepare_mujoco.build_scene(config, "sensor_lab", output)


if __name__ == "__main__":
    unittest.main()
