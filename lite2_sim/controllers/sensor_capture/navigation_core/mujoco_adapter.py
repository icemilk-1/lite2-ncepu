"""Ideal multilayer ray observations using the shared navigation contract."""

import math

import mujoco
import numpy as np

from .contracts import NavigationObservation


class MujocoLidarObservationAdapter:
    def __init__(self, model, data, settings, reference_height_m):
        self.model = model
        self.data = data
        self.settings = settings
        self.reference_height_m = float(reference_height_m)
        self.site = model.site("front_lidar").id
        self.layers = int(settings.get("number_of_layers", 1))
        self.beams = int(settings["horizontal_resolution"])
        if self.layers < 1 or self.beams < 2:
            raise ValueError("LiDAR requires positive layers and at least two horizontal beams.")
        fov = float(settings["field_of_view_rad"])
        full_circle = math.isclose(fov, 2 * math.pi, abs_tol=1e-6)
        horizontal = np.linspace(-fov / 2, fov / 2, self.beams, endpoint=not full_circle)
        vertical_fov = float(settings["vertical_field_of_view_rad"])
        self.vertical = np.linspace(-vertical_fov / 2, vertical_fov / 2, self.layers)
        if self.layers == 1:
            self.vertical[:] = 0
        self.directions = np.stack(np.broadcast_arrays(
            np.cos(self.vertical[:, None]) * np.cos(horizontal),
            np.cos(self.vertical[:, None]) * np.sin(horizontal),
            np.sin(self.vertical[:, None]) * np.ones(self.beams)), axis=-1)
        self.random = np.random.default_rng(int(settings.get("random_seed", 42)))
        # A blocked beam models a dirty lens or a permanent occluder: no return at
        # any layer, while the beam keeps its identity in the organized cloud.
        block_probability = float(settings.get("beam_block_probability", 0.0) or 0.0)
        if not 0.0 <= block_probability <= 1.0:
            raise ValueError("LiDAR beam_block_probability must be in [0, 1].")
        self.blocked_beams = (self.random.random(self.beams) < block_probability
                              if block_probability > 0.0 else np.zeros(self.beams, dtype=bool))
        self.raw = None

    def read(self, timestamp_s, capture_path=None):
        settings = self.settings
        rotation = self.data.site_xmat[self.site].reshape(3, 3)
        world_rays = np.ascontiguousarray((self.directions @ rotation.T).reshape(-1))
        count = self.layers * self.beams
        distances = np.full(count, -1.0)
        geom_ids = np.full(count, -1, dtype=np.int32)
        groups = np.array([0, 0, 0, 1, 0, 0], dtype=np.uint8)
        mujoco.mj_multiRay(self.model, self.data, self.data.site_xpos[self.site],
                          world_rays, groups, True, -1, geom_ids, distances,
                          None, count, 1e6)
        distances = distances.reshape(self.layers, self.beams)
        minimum, maximum = settings["min_range_m"], settings["max_range_m"]
        no_hit = (geom_ids.reshape(self.layers, self.beams) == -1) & (distances == -1)
        finite_hit = np.isfinite(distances) & (distances >= 0)
        beyond = finite_hit & (distances > maximum)
        hit = finite_hit & (distances >= minimum) & ~beyond
        range_bias = float(settings.get("range_bias_m", 0.0) or 0.0)
        measured = distances.copy()
        if hit.any():
            # A constant additive bias is a low-cost stand-in for a calibration
            # error; the zero-mean term models the ranging noise floor.
            measured[hit] += range_bias
        measured[hit] += self.random.normal(0, float(settings.get("noise_m", 0)), int(hit.sum()))
        dropout = float(settings.get("dropout_probability", 0.0))
        if not 0.0 <= dropout < 1.0:
            raise ValueError("LiDAR dropout_probability must be in [0, 1).")
        dropped = hit & (self.random.random(hit.shape) < dropout)
        if self.blocked_beams.any():
            dropped |= np.broadcast_to(self.blocked_beams[None, :], hit.shape)
        hit[dropped] = False
        no_hit[dropped] = True
        hit &= (measured >= minimum) & (measured <= maximum)
        valid = hit | no_hit | beyond
        ranges = np.where(hit, measured, maximum)
        ranges[~valid] = np.nan
        status = np.zeros(ranges.shape, dtype=np.uint8)
        status[hit], status[no_hit], status[beyond] = 1, 2, 3
        xyz = self.directions * ranges[:, :, None]
        self.raw = {"xyz_m": xyz.astype(np.float32), "range_image_m": ranges.astype(np.float32),
                    "raw_ray_distance_m": distances, "valid": valid, "hit": hit,
                    "status": status, "geom_id": geom_ids.reshape(ranges.shape)}
        if capture_path is not None:
            np.savez_compressed(capture_path, **self.raw)

        heights = self.reference_height_m + xyz[:, :, 2]
        obstacle = hit & (heights >= settings["obstacle_height_min_m"]) & (
            heights <= settings["obstacle_height_max_m"])
        horizontal_distance = np.hypot(xyz[:, :, 0], xyz[:, :, 1])
        points = []
        for index in range(self.beams):
            layers = np.flatnonzero(obstacle[:, index])
            if layers.size:
                layer = int(layers[np.argmin(horizontal_distance[layers, index])])
                points.append((*xyz[layer, index, :2], True))
            elif valid[self.layers // 2, index]:
                points.append((*xyz[self.layers // 2, index, :2], False))
        metadata = {
            "horizontal_beams_per_layer": self.beams, "vertical_layers": self.layers,
            "frame_id": "front_lidar", "timestamp_s": float(timestamp_s),
            "sensor_model": settings["model_status"],
            "self_filter": "environment_collision_geoms_only_group_3",
            "status_codes": {"0": "invalid", "1": "hit", "2": "no_hit", "3": "beyond_range"},
            "status_counts": {str(code): int(np.count_nonzero(status == code)) for code in range(4)},
            "range_limits_m": [minimum, maximum], "frequency_hz": settings["frequency_hz"],
            "noise_m": float(settings.get("noise_m", 0)),
            "range_bias_m": float(settings.get("range_bias_m", 0.0) or 0.0),
            "dropout_probability": dropout,
            "beam_block_probability": float(settings.get("beam_block_probability", 0.0) or 0.0),
            "blocked_beams": int(np.count_nonzero(self.blocked_beams)),
            "scan_latency_s": float(settings.get("scan_latency_s", 0.0) or 0.0),
            "projection_reference_height_world_m": self.reference_height_m,
            "projection_status": "fixed_fixture_reference_height",
        }
        return NavigationObservation(timestamp_s, "front_lidar", points,
                                     len(points) >= 3, "mujoco_ideal_multilayer_rays", metadata)
