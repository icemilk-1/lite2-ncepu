"""Webots-specific observation and kinematic execution adapters."""

import math

import numpy as np

from navigation_core.contracts import NavigationObservation


def _wrap_pi(values):
    return (values + math.pi) % (2 * math.pi) - math.pi


def _scan_data(lidar, settings, sensor_height_world_m, capture_path=None):
    layer_count = int(lidar.getNumberOfLayers())
    max_range = settings["max_range_m"]
    min_range = settings["min_range_m"]
    count = int(lidar.getHorizontalResolution())
    clouds = [lidar.getLayerPointCloud(layer) for layer in range(layer_count)]
    xyz = np.asarray(
        [[[float(p.x), float(p.y), float(p.z)] for p in cloud[:count]] for cloud in clouds],
        dtype=np.float32,
    )
    # R2025a's Python getLayerRangeImage calls the whole-image C function.
    ranges_array = np.asarray(lidar.getRangeImage(), dtype=np.float32).reshape(layer_count, count)
    if capture_path is not None:
        np.savez_compressed(capture_path, xyz_m=xyz, range_image_m=ranges_array)
    horizontal_ranges = np.hypot(xyz[:, :, 0], xyz[:, :, 1])
    range_3d = np.sqrt(horizontal_ranges ** 2 + xyz[:, :, 2] ** 2)
    valid_cloud = np.isfinite(xyz).all(axis=2) & (range_3d > min_range) & (range_3d <= max_range + 1e-3)
    self_hits = np.zeros(valid_cloud.shape, dtype=bool)
    if "self_filter_body_box_m" in settings:
        lower, upper = np.asarray(settings["self_filter_body_box_m"], dtype=np.float32)
        body_points = xyz + np.asarray(settings["mount_xyz_m"], dtype=np.float32)
        self_hits = valid_cloud & (body_points >= lower).all(axis=2) & (body_points <= upper).all(axis=2)
    valid_cloud &= ~self_hits
    valid_horizontal = valid_cloud & (horizontal_ranges > min_range) & (horizontal_ranges < max_range - 0.05)
    obstacle_heights = sensor_height_world_m + xyz[:, :, 2]
    obstacle_hits = valid_horizontal & (obstacle_heights >= settings["obstacle_height_min_m"]) & (obstacle_heights <= settings["obstacle_height_max_m"])

    fov = float(lidar.getFov())
    step_magnitude = fov / count if fov >= 2 * math.pi - 1e-4 else fov / max(1, count - 1)
    angle_by_index = {}
    for layer in range(layer_count):
        for index in np.flatnonzero(valid_horizontal[layer]):
            angle_by_index.setdefault(
                int(index), math.atan2(float(xyz[layer, index, 1]), float(xyz[layer, index, 0])),
            )
    hit_indices = sorted(angle_by_index)
    hit_angles = [angle_by_index[index] for index in hit_indices]
    rates = []
    for i in range(len(hit_indices) - 1):
        index_delta = hit_indices[i + 1] - hit_indices[i]
        if 0 < index_delta <= 8:
            angle_delta = float(_wrap_pi(hit_angles[i + 1] - hit_angles[i]))
            rate = angle_delta / index_delta
            if step_magnitude * 0.45 <= abs(rate) <= step_magnitude * 1.8:
                rates.append(rate)
    if rates:
        step = math.copysign(step_magnitude, float(np.median(rates)))
        offsets = np.asarray(hit_angles) - step * np.asarray(hit_indices)
        offset = math.atan2(float(np.sin(offsets).mean()), float(np.cos(offsets).mean()))
        angle_source = "inferred_from_webots_lidar_point_cloud"
    else:
        step = step_magnitude
        offset = -fov / 2
        angle_source = "fallback_device_fov_order"
    beam_angles = offset + step * np.arange(count)

    points = []
    clearing_layer = layer_count // 2
    mapped_height_hits = 0
    for index in range(count):
        layer_hits = np.flatnonzero(obstacle_hits[:, index])
        if layer_hits.size:
            layer = int(layer_hits[np.argmin(horizontal_ranges[layer_hits, index])])
            points.append((float(xyz[layer, index, 0]), float(xyz[layer, index, 1]), True))
            mapped_height_hits += 1
            continue
        # Clear using a near-horizontal ray. Upper-layer sky returns cannot
        # establish free ground behind a lower-layer obstacle or self return.
        if valid_cloud[clearing_layer, index]:
            points.append((float(xyz[clearing_layer, index, 0]), float(xyz[clearing_layer, index, 1]), False))
        elif np.isposinf(ranges_array[clearing_layer, index]) and not self_hits[clearing_layer, index]:
            points.append((max_range * math.cos(beam_angles[index]), max_range * math.sin(beam_angles[index]), False))

    finite_ranges = np.isfinite(ranges_array)
    finite_values = ranges_array[finite_ranges]
    sample_layers = sorted(set([0, layer_count // 2, layer_count - 1]))
    sample_indices = sorted(set([0, count // 4, count // 2, 3 * count // 4, count - 1]))
    metadata = {
        "horizontal_beams_per_layer": count,
        "vertical_layers": layer_count,
        "device_field_of_view_rad": fov,
        "vertical_field_of_view_rad": float(lidar.getVerticalFov()),
        "point_cloud_returns": int(valid_cloud.sum()),
        "self_filtered_returns": int(self_hits.sum()),
        "self_filter_body_box_m": settings.get("self_filter_body_box_m"),
        "clearing_layer": clearing_layer,
        "range_image_source": "whole_image_reshaped_by_layer",
        "obstacle_height_hits": int(obstacle_hits.sum()),
        "mapped_2d_obstacle_beams": mapped_height_hits,
        "scan_ready": int(valid_cloud.sum()) >= 3,
        "obstacle_height_band_m": [settings["obstacle_height_min_m"], settings["obstacle_height_max_m"]],
        "range_image_finite_fraction": float(finite_ranges.mean()),
        "angle_source": angle_source,
        "first_beam_angle_rad": float(beam_angles[0]) if count else None,
        "angle_step_rad": float(step),
        "range_min_observed_m": float(np.min(finite_values)) if finite_values.size else None,
        "range_max_observed_m": float(np.max(finite_values)) if finite_values.size else None,
        "point_cloud_samples": [
            {"layer": layer, "index": index, "x_m": float(xyz[layer, index, 0]),
             "y_m": float(xyz[layer, index, 1]), "z_m": float(xyz[layer, index, 2]),
             "height_world_m": float(obstacle_heights[layer, index]),
             "range_m": float(range_3d[layer, index]), "range_image_m": float(ranges_array[layer, index])}
            for layer in sample_layers for index in sample_indices
        ],
    }
    for sample in metadata["point_cloud_samples"]:
        for key, value in sample.items():
            if isinstance(value, float) and not math.isfinite(value):
                sample[key] = None
    return points, metadata



class WebotsLidarObservationAdapter(object):
    def __init__(self, lidar, settings, sensor_height_world_m):
        self.lidar = lidar
        self.settings = settings
        self.sensor_height_world_m = float(sensor_height_world_m)

    def read(self, timestamp_s, capture_path=None):
        points, metadata = _scan_data(
            self.lidar, self.settings, self.sensor_height_world_m, capture_path,
        )
        observation = NavigationObservation(
            timestamp_s, "front_lidar", points, metadata["scan_ready"],
            "webots_multilayer_lidar", metadata,
        )
        return observation


class WebotsKinematicExecutor(object):
    """Moves the simulated fixture; its pose never enters navigation decisions."""

    def __init__(self, fixture, ground_z_m, grid):
        self.translation_field = fixture.getField("translation")
        self.rotation_field = fixture.getField("rotation")
        if self.translation_field is None or self.rotation_field is None:
            raise RuntimeError("The robot fixture has no mutable pose fields.")
        self.ground_z_m = float(ground_z_m)
        self.grid = grid

    def step(self, truth_pose, command, now_s, dt_s, collision_map):
        v, w = command.linear_x_m_s, command.angular_z_rad_s
        if command.expired(now_s):
            v, w = 0.0, 0.0
        old_xy = tuple(truth_pose[:2])
        mid_yaw = truth_pose[2] + w * dt_s * 0.5
        next_x = truth_pose[0] + v * math.cos(mid_yaw) * dt_s
        next_y = truth_pose[1] + v * math.sin(mid_yaw) * dt_s
        collision = collision_map is not None and self.grid.segment_blocked(
            truth_pose[:2], (next_x, next_y), collision_map,
        )
        if collision:
            v = 0.0
        else:
            truth_pose[0], truth_pose[1] = next_x, next_y
        truth_pose[2] = (truth_pose[2] + w * dt_s + math.pi) % (2.0 * math.pi) - math.pi
        self.translation_field.setSFVec3f([truth_pose[0], truth_pose[1], self.ground_z_m])
        self.rotation_field.setSFRotation([0.0, 0.0, 1.0, truth_pose[2]])
        distance = math.hypot(truth_pose[0] - old_xy[0], truth_pose[1] - old_xy[1])
        return (v, w), distance, bool(collision)
