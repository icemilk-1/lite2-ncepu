"""Observed terrain and conservative body-clearance mapping from organized clouds."""

import math

import numpy as np

from navigation_core.contracts import NavigationObservation
from navigation_core.grid import OccupancyGrid


def tilt_rotation(roll, pitch):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    return np.array([[cp, sp * sr, sp * cr], [0, cr, -sr], [-sp, cp * sr, cp * cr]])


def project_cloud(raw, settings, motion, base_height_m):
    """Estimate surface normals from neighboring returns, without geometry identities."""
    rotation = tilt_rotation(motion.roll_rad, motion.pitch_rad)
    xyz = np.asarray(raw["xyz_m"], dtype=float) @ rotation.T
    hit = raw["hit"]
    valid = raw["valid"]
    mount = rotation @ np.asarray(settings["mount_xyz_m"])
    z = xyz[:, :, 2] + mount[2] + base_height_m
    xyz[:, :, :2] += mount[:2] - np.asarray(settings["mount_xy_m"])
    # Central differences on organized beams provide a sensor-derived surface normal.
    azimuth = np.roll(xyz, -1, axis=1) - np.roll(xyz, 1, axis=1)
    before, after = np.roll(hit, 1, axis=0), np.roll(hit, -1, axis=0)
    before[0], after[-1] = False, False
    backward = xyz - np.roll(xyz, 1, axis=0)
    forward = np.roll(xyz, -1, axis=0) - xyz
    elevation = np.where((before & after)[:, :, None], backward + forward,
                         np.where(before[:, :, None], backward, forward))
    normals = np.cross(azimuth, elevation)
    normals *= np.where(np.sum(normals * xyz, axis=2) > 0, -1., 1.)[:, :, None]
    magnitude = np.linalg.norm(normals, axis=2)
    vertical_normal = normals[:, :, 2] / np.maximum(magnitude, 1e-9)
    neighbors = hit & np.roll(hit, 1, axis=1) & np.roll(hit, -1, axis=1)
    neighbors &= before | after
    unit_normals = normals / np.maximum(magnitude[:, :, None], 1e-9)
    plane_error = np.zeros(hit.shape)
    for dy, dx in ((-2, 0), (-1, 0), (1, 0), (2, 0), (0, -1), (0, 1)):
        neighbor_hit = np.roll(hit, (dy, dx), axis=(0, 1))
        if dy > 0:
            neighbor_hit[:dy, :] = False
        elif dy < 0:
            neighbor_hit[dy:, :] = False
        difference = np.roll(xyz, (dy, dx), axis=(0, 1)) - xyz
        residual = np.abs(np.sum(unit_normals * difference, axis=2))
        plane_error = np.maximum(plane_error, np.where(neighbor_hit, residual, 0.0))
    normal_ground = neighbors & (plane_error < 0.02)
    normal_ground &= vertical_normal >= math.cos(math.radians(settings.get("max_slope_deg", 15)))
    distance_xy = np.linalg.norm(xyz[:, :, :2], axis=2)
    # Ground heights must be compatible with a continuous slope from the current support.
    allowance = settings.get("max_step_m", 0.08) + distance_xy * math.tan(math.radians(settings.get("max_slope_deg", 15)))
    ground = hit & (normal_ground | (np.abs(z - motion.ground_height_m) < 0.025))
    ground &= np.abs(z - motion.ground_height_m) <= allowance
    body_top = settings.get("body_clearance_height_m", 0.60)
    obstacle = hit & ~ground & (z > motion.ground_height_m + 0.04) & (z < motion.ground_height_m + body_top)
    points = []
    for beam in range(xyz.shape[1]):
        blocking = np.flatnonzero(obstacle[:, beam])
        if blocking.size:
            layer = int(blocking[np.argmin(distance_xy[blocking, beam])])
            points.append((xyz[layer, beam, 0], xyz[layer, beam, 1], True))
            continue
        # Only rays near horizontal can certify body-height free space. Floor hits
        # stop clearing at their observed endpoint, even when other layers see farther.
        candidates = np.flatnonzero(valid[:, beam] & (np.abs(xyz[:, beam, 2]) < 0.12))
        if candidates.size:
            layer = int(candidates[np.argmax(distance_xy[candidates, beam])])
            points.append((xyz[layer, beam, 0], xyz[layer, beam, 1], False))
        elif ground[:, beam].any():
            candidates = np.flatnonzero(ground[:, beam])
            layer = int(candidates[np.argmax(distance_xy[candidates, beam])])
            points.append((xyz[layer, beam, 0], xyz[layer, beam, 1], False))
    terrain_points = np.column_stack((xyz[ground, :2], z[ground]))
    slopes = np.where(normal_ground[ground], np.degrees(np.arccos(np.clip(vertical_normal[ground], -1, 1))), 0.0)
    ground_normals = unit_normals[ground].copy()
    ground_normals[~normal_ground[ground]] = [0., 0., 1.]
    overhead = hit & neighbors & (vertical_normal < -0.99) & (z >= motion.ground_height_m + 0.04)
    overhead &= (np.abs(z - np.roll(z, 1, axis=1)) < 0.02) & (np.abs(z - np.roll(z, -1, axis=1)) < 0.02)
    overhead_points = np.column_stack((xyz[overhead, :2], z[overhead]))
    metadata = {"terrain_points_sensor_level_m": terrain_points,
                "terrain_slopes_deg": slopes,
                "terrain_normals_sensor_level": ground_normals,
                "obstacle_points_sensor_level_m": np.column_stack((xyz[obstacle, :2], z[obstacle])),
                "sensor_height_m": float(mount[2] + base_height_m),
                "measured_support_height_m": float(motion.ground_height_m),
                "raw_frame_id": "front_lidar",
                "projection_frame_axes": "x_forward_y_left_z_gravity_up_with_body_yaw",
                "nominal_origin_body_xy_m": list(settings["mount_xy_m"]),
                "surface_points_sensor_level_m": overhead_points,
                "projection_status": "imu_tilt_compensated_measured_height",
                "ground_return_count": int(ground.sum()), "obstacle_return_count": int(obstacle.sum())}
    return NavigationObservation(motion.timestamp_s, "front_lidar_level", points, motion.valid and len(points) >= 8,
                                 "mujoco_organized_cloud_projection", metadata)


class TerrainGrid(OccupancyGrid):
    def __init__(self, field_size, resolution, settings):
        super(TerrainGrid, self).__init__(field_size, resolution)
        self.settings = settings
        self.height_m = np.full_like(self.log_odds, np.nan)
        self.slope_deg = np.full_like(self.log_odds, np.nan)
        self.clearance_m = np.full_like(self.log_odds, np.nan)
        self.surface_z = np.full_like(self.log_odds, np.nan)
        self.last_hit_s = np.full_like(self.log_odds, -np.inf)
        self.clock_s = 0.0
        self.sensor_height_m = 0.0
        self.obstacle_top_m = np.full_like(self.log_odds, -np.inf)
        self.terrain_hazard = np.zeros_like(self.log_odds, dtype=bool)

    def integrate(self, *args, **kwargs):
        previous = self.log_odds.copy()
        count = super(TerrainGrid, self).integrate(*args, **kwargs)
        # Elevated rays passing above an observed obstacle cannot remove it.
        protected = (previous >= 0.7) & (self.obstacle_top_m < self.sensor_height_m - 0.10)
        self.log_odds[protected] = np.maximum(previous[protected], self.log_odds[protected])
        return count

    def _cells(self, points, pose, extrinsic):
        if len(points) == 0:
            return np.empty((0, 3)), np.empty(0, dtype=int), np.empty(0, dtype=int)
        c, s = math.cos(pose[2]), math.sin(pose[2])
        points = np.asarray(points)
        x = pose[0] + c * (points[:, 0] + extrinsic[0]) - s * (points[:, 1] + extrinsic[1])
        y = pose[1] + s * (points[:, 0] + extrinsic[0]) + c * (points[:, 1] + extrinsic[1])
        col = np.floor((x - self.x_min) / self.resolution).astype(int)
        row = np.floor((y - self.y_min) / self.resolution).astype(int)
        inside = (col >= 0) & (col < self.width) & (row >= 0) & (row < self.height)
        return points[inside], col[inside], row[inside]

    def integrate_terrain(self, observation, pose, extrinsic):
        self.clock_s = observation.timestamp_s
        points, col, row = self._cells(observation.metadata.get("terrain_points_sensor_level_m", []), pose, extrinsic)
        self.height_m[row, col] = points[:, 2]
        # Slopes are derived from the observed surface, and stay unknown elsewhere.
        slopes = np.asarray(observation.metadata.get("terrain_slopes_deg", []))
        if len(slopes) == len(points):
            self.slope_deg[row, col] = slopes
        ground_points = observation.metadata.get("terrain_points_sensor_level_m", [])
        ground_normals = observation.metadata.get("terrain_normals_sensor_level", [])
        if len(ground_points) and len(ground_points) == len(ground_normals):
            samples, cols, rows = self._cells(np.column_stack((ground_points, ground_normals)), pose, extrinsic)
            c, s = math.cos(pose[2]), math.sin(pose[2])
            nx = c * samples[:, 3] - s * samples[:, 4]
            ny = s * samples[:, 3] + c * samples[:, 4]
            nz = np.maximum(samples[:, 5], 0.9)
            # Only fresh, measured local planes fill adjacent cells. Previously
            # interpolated cells never seed further extrapolation.
            for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                rr, cc = rows + dy, cols + dx
                inside = (rr >= 0) & (rr < self.height) & (cc >= 0) & (cc < self.width)
                self.height_m[rr[inside], cc[inside]] = (samples[:, 2] - self.resolution * (nx*dx + ny*dy) / nz)[inside]
        for dy, dx in ((1, 0), (0, 1)):
            for gap in range(1, 5):
                adjacent = np.roll(self.height_m, (dy*gap, dx*gap), axis=(0, 1))
                edge = np.isfinite(self.height_m) & np.isfinite(adjacent)
                if dy:
                    edge[:gap, :] = False
                if dx:
                    edge[:, :gap] = False
                allowed = self.settings["max_step_m"] + gap*self.resolution*math.tan(math.radians(self.settings["max_slope_deg"]))
                edge &= np.abs(self.height_m - adjacent) > allowed
                for offset in range(gap+1):
                    self.terrain_hazard |= np.roll(edge, (-dy*offset, -dx*offset), axis=(0, 1))
        points, col, row = self._cells(observation.metadata.get("surface_points_sensor_level_m", []), pose, extrinsic)
        self.surface_z[row, col] = points[:, 2]
        points, col, row = self._cells(observation.metadata.get("obstacle_points_sensor_level_m", []), pose, extrinsic)
        np.maximum.at(self.obstacle_top_m, (row, col), points[:, 2])
        known = np.isfinite(self.height_m) & np.isfinite(self.surface_z)
        self.clearance_m[known] = self.surface_z[known] - self.height_m[known]
        # Expire unconfirmed obstacles, then require fresh free rays before reuse.
        hits = np.asarray([(x, y, 0) for x, y, hit in observation.points if hit])
        _, col, row = self._cells(hits, pose, extrinsic)
        self.last_hit_s[row, col] = self.clock_s
        stale = (self.log_odds >= 0.7) & (self.clock_s - self.last_hit_s > self.settings.get("obstacle_memory_s", 5.0))
        stale &= self.obstacle_top_m >= self.sensor_height_m - 0.10
        self.log_odds[stale] = 0.0
        support_height = observation.metadata.get("measured_support_height_m", 0.0)
        proven_support = np.isfinite(self.height_m) & np.isfinite(self.obstacle_top_m)
        proven_support &= np.abs(self.height_m - self.obstacle_top_m) < 0.035
        proven_support &= np.abs(self.height_m - support_height) <= self.settings["max_step_m"]
        self.log_odds[proven_support] = np.minimum(self.log_odds[proven_support], -0.7)
        low_roof = known & (self.clearance_m > 0.08) & (self.clearance_m < self.settings["body_clearance_height_m"])
        steep = np.isfinite(self.slope_deg) & (self.slope_deg > self.settings["max_slope_deg"])
        hazards = low_roof | steep
        self.log_odds[hazards] = np.maximum(self.log_odds[hazards], 0.9)

    def inflated(self, radius_m, occupied_threshold):
        result = super(TerrainGrid, self).inflated(radius_m, occupied_threshold)
        # Surface transitions already describe measured support cells. Keep their
        # uncertainty separate from the occupied-object footprint inflation.
        result |= self.terrain_hazard
        return result
