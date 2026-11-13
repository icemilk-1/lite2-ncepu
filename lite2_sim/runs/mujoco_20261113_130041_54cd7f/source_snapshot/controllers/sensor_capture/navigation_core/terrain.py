"""Observed terrain and conservative body-clearance mapping from organized clouds."""

import math

import numpy as np

from navigation_core.contracts import NavigationObservation
from navigation_core.grid import OccupancyGrid


def tilt_rotation(roll, pitch):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    return np.array([[cp, sp * sr, sp * cr], [0, cr, -sr], [-sp, cp * sr, cp * cr]])


def _shift(target, source, sy, sx, fill):
    """target[r, c] = source[r - sy, c - sx] with ``fill`` outside the source."""
    target[:] = fill
    rows, columns = source.shape
    target[max(0, sy):rows - max(0, -sy), max(0, sx):columns - max(0, -sx)] = \
        source[max(0, -sy):rows - max(0, sy), max(0, -sx):columns - max(0, sx)]


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
    normal_ground = neighbors & (vertical_normal >= math.cos(math.radians(settings.get("max_slope_deg", 15))))
    distance_xy = np.linalg.norm(xyz[:, :, :2], axis=2)
    # Ground heights must be compatible with a continuous slope from the current support.
    allowance = settings.get("max_step_m", 0.08) + distance_xy * math.tan(math.radians(settings.get("max_slope_deg", 15)))
    # A range-noise return on the floor must not read as a body-height obstacle.
    flat_tolerance = max(0.025, 2.0 * float(settings.get("noise_m", 0.0) or 0.0))
    ground = hit & (normal_ground | (np.abs(z - motion.ground_height_m) < flat_tolerance))
    ground &= np.abs(z - motion.ground_height_m) <= allowance
    body_top = settings.get("body_clearance_height_m", 0.60)
    obstacle_height = flat_tolerance
    # Every observed riser remains an obstacle, including low stones and lips.
    # Smooth support surfaces still use the normal/height classification above.
    obstacle = hit & ~ground & (z > motion.ground_height_m + obstacle_height) & (z < motion.ground_height_m + body_top)
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
        self.height_updated_s = np.full_like(self.log_odds, -np.inf)
        self.slope_deg = np.full_like(self.log_odds, np.nan)
        self.clearance_m = np.full_like(self.log_odds, np.nan)
        self.surface_z = np.full_like(self.log_odds, np.nan)
        self.last_hit_s = np.full_like(self.log_odds, -np.inf)
        self.clock_s = 0.0
        self.sensor_height_m = 0.0
        self.obstacle_top_m = np.full_like(self.log_odds, -np.inf)
        self.terrain_hazard = np.zeros_like(self.log_odds, dtype=bool)
        self.height_discontinuities = np.zeros_like(self.log_odds, dtype=bool)
        self.hit_scan = np.full_like(self.log_odds, -1000, dtype=np.int32)
        self.scan_index = 0
        self.hazard_evidence = np.zeros_like(self.log_odds, dtype=np.float32)
        self.explored_surface = np.zeros_like(self.log_odds, dtype=bool)
        self.support_height_m = 0.0

    def traversable_step_mask(self):
        """Obstacle crossing is disabled for the avoidance-only navigation policy."""
        return np.zeros_like(self.log_odds, dtype=bool)

    def integrate(self, *args, **kwargs):
        previous = self.log_odds.copy()
        self.scan_index += 1
        count = super(TerrainGrid, self).integrate(*args, **kwargs)
        threshold = self.settings["occupied_log_odds_threshold"]
        # A single scan may place a range-noise return on the floor in front of
        # the robot. One frame is not enough evidence to block planning: keep a
        # first-time hit below the occupancy threshold until the same cell is hit
        # again within a few scans (the robot moves and rotates between scans, so
        # requiring the immediately previous scan would never confirm a wall).
        hits = getattr(self, "last_hit_mask", None)
        if hits is not None and hits.any():
            window = int(self.settings.get("hit_confirmation_scans", 4))
            confirmed = hits & (self.hit_scan >= self.scan_index - window)
            self.log_odds[hits & ~confirmed] = np.minimum(
                self.log_odds[hits & ~confirmed], threshold - 0.35)
            self.hit_scan[hits] = self.scan_index
        # Elevated rays passing above an observed obstacle cannot remove it.
        protected = (previous >= threshold) & (self.obstacle_top_m < self.sensor_height_m - 0.10)
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
        self.height_updated_s[row, col] = self.clock_s
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
                self.height_updated_s[rr[inside], cc[inside]] = self.clock_s
        # Re-evaluate discontinuities after fresh heights replace old samples.
        # Accumulating this mask permanently preserves transient ramp edges.
        self.terrain_hazard.fill(False)
        fresh = self.clock_s - self.height_updated_s <= float(self.settings.get("height_observation_timeout_s", 0.5))
        for dy, dx in ((1, 0), (0, 1)):
            for gap in range(1, 5):
                adjacent = np.roll(self.height_m, (dy*gap, dx*gap), axis=(0, 1))
                edge = np.isfinite(self.height_m) & np.isfinite(adjacent)
                edge &= fresh & np.roll(fresh, (dy*gap, dx*gap), axis=(0, 1))
                if dy:
                    edge[:gap, :] = False
                if dx:
                    edge[:, :gap] = False
                allowed = self.settings.get("surface_height_tolerance_m", 0.02) + gap*self.resolution*math.tan(math.radians(self.settings["max_slope_deg"]))
                difference = self.height_m - adjacent
                # Put a raised-object hazard on the higher surface. Spreading
                # it across measured floor cells shrinks a narrow corridor by
                # the sampling gap before body inflation is even applied.
                self.terrain_hazard |= edge & (difference > allowed)
                self.terrain_hazard |= np.roll(edge & (difference < -allowed),
                                               (-dy*gap, -dx*gap), axis=(0, 1))
        self.height_discontinuities[:] = self.terrain_hazard
        # A descending support edge constrains the centre path. Ground already
        # below the body is distinct from a raised object in its footprint.
        support_height = float(observation.metadata.get("measured_support_height_m", 0.0))
        self.terrain_hazard &= self.height_m > support_height + self.settings.get("surface_height_tolerance_m", 0.02)
        # Slope is measured on the height field, not from point normals: at
        # grazing incidence the layer-to-layer difference is not a surface
        # tangent and a 12 degree ramp would otherwise read as too steep.
        slope_ratio = np.zeros(self.height_m.shape, dtype=self.height_m.dtype)
        for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):
            span = self.resolution * math.hypot(dy, dx)
            for gap in (3, 4):
                adjacent = np.roll(self.height_m, (dy*gap, dx*gap), axis=(0, 1))
                edge = np.isfinite(self.height_m) & np.isfinite(adjacent)
                if dy:
                    edge[:gap, :] = False
                if dx:
                    edge[:, :gap] = False
                ratio = np.zeros(self.height_m.shape, dtype=self.height_m.dtype)
                np.divide(np.abs(self.height_m - adjacent), gap*span, out=ratio, where=edge)
                np.maximum(slope_ratio, ratio, out=slope_ratio)
        self.slope_deg = np.where(np.isfinite(self.height_m), np.degrees(np.arctan(slope_ratio)), np.nan)
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
        threshold = self.settings["occupied_log_odds_threshold"]
        stale = (self.log_odds >= threshold) & (self.clock_s - self.last_hit_s > self.settings.get("obstacle_memory_s", 5.0))
        stale &= self.obstacle_top_m >= self.sensor_height_m - 0.10
        self.log_odds[stale] = 0.0
        support_height = observation.metadata.get("measured_support_height_m", 0.0)
        self.support_height_m = float(support_height)
        proven_support = np.isfinite(self.height_m) & np.isfinite(self.obstacle_top_m)
        proven_support &= np.abs(self.height_m - self.obstacle_top_m) < 0.035
        proven_support &= np.abs(self.height_m - support_height) <= self.settings.get("surface_height_tolerance_m", 0.02)
        self.log_odds[proven_support] = np.minimum(self.log_odds[proven_support], -0.7)
        low_roof = known & (self.clearance_m > 0.08) & (self.clearance_m < self.settings["body_clearance_height_m"])
        # Steepness is handled by the support-reachability test below: a slope
        # that exceeds the limit is simply never cleared. A separate per-cell
        # slope gate would only re-add grazing-incidence false positives.
        hazards = low_roof
        # Hazard evidence accumulates per scan so one noisy height sample cannot
        # block the current cell; a real clearance limit confirms fast.
        self.hazard_evidence[hazards] = np.minimum(
            threshold + 0.2, self.hazard_evidence[hazards] + 0.45)
        self.hazard_evidence[~hazards] = np.maximum(0.0, self.hazard_evidence[~hazards] - 0.15)
        self.log_odds = np.maximum(self.log_odds, self.hazard_evidence)
        self._clear_continuous_surface(pose, support_height)

    def _clear_continuous_surface(self, pose, support_height):
        """Trust smooth surfaces that stay connected to the measured support.

        Grazing beams on a ramp make the per-point surface normal unreliable, so
        parts of an ascending ramp get flagged as obstacles and the robot refuses
        to climb it. Only cells reachable from the robot's own support through a
        chain of slope-limited steps are cleared: rubble crowns and wall tops are
        locally smooth but are not reachable from the floor, so they stay blocked.
        """
        if not np.isfinite(self.height_m).any():
            return
        slope_tan = math.tan(math.radians(self.settings["max_slope_deg"]))
        # Height samples carry measurement and pose noise; a fixed metric
        # tolerance keeps a 12 degree ramp drivable without turning rubble into
        # free space.
        tolerance = self.settings.get("surface_height_tolerance_m", 0.02)
        centre = self.world_to_cell(*pose[:2])
        if centre is None:
            return
        rows, columns = self.height_m.shape
        radius = max(4, int(math.ceil(float(self.settings.get("surface_clear_radius_m", 2.5)) / self.resolution)))
        r0, r1 = max(0, centre[1] - radius), min(rows, centre[1] + radius + 1)
        c0, c1 = max(0, centre[0] - radius), min(columns, centre[0] + radius + 1)
        height = self.height_m[r0:r1, c0:c1]
        known = np.isfinite(height)
        obstacle_top = self.obstacle_top_m[r0:r1, c0:c1]
        # A cell can only be re-trusted when no return rises clearly above the
        # observed surface: grazing ramp returns sit on the surface, a real wall
        # or rubble face stands above it.
        flat_enough = (obstacle_top - height) < self.settings.get("surface_obstacle_tolerance_m", 0.10)
        drivable = known & flat_enough
        drivable &= self.hazard_evidence[r0:r1, c0:c1] < self.settings["occupied_log_odds_threshold"]
        reachable = np.zeros_like(drivable)
        start = (centre[1] - r0, centre[0] - c0)
        if 0 <= start[0] < height.shape[0] and 0 <= start[1] < height.shape[1] and drivable[start]:
            reachable[start] = True
        else:
            # The robot's own cell may lack a height sample; seed every nearby
            # drivable cell whose height matches the measured support.
            local = np.zeros_like(drivable)
            local[max(0, start[0]-2):start[0]+3, max(0, start[1]-2):start[1]+3] = True
            reachable[local & drivable & (np.abs(height - support_height) <= tolerance)] = True
        if not reachable.any():
            return
        shifted_height = np.empty(height.shape, dtype=height.dtype)
        shifted_reachable = np.empty(height.shape, dtype=bool)
        connections = []
        for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):
            limit = slope_tan * self.resolution * math.hypot(dy, dx) + tolerance
            for sign in (1, -1):
                sy, sx = dy*sign, dx*sign
                _shift(shifted_height, height, sy, sx, np.nan)
                connected = drivable & np.isfinite(shifted_height) & (np.abs(shifted_height-height) <= limit)
                connections.append((sy, sx, connected))
        # Every link must be a measured adjacent support transition. Long jumps
        # could skip a rock's vertical face and incorrectly clear its flat top.
        for _ in range(height.shape[0] + height.shape[1]):
            grown = reachable.copy()
            for sy, sx, connected in connections:
                _shift(shifted_reachable, reachable, sy, sx, False)
                grown |= shifted_reachable & connected
            if np.array_equal(grown, reachable):
                break
            reachable = grown
        if reachable.any():
            window = self.log_odds[r0:r1, c0:c1]
            window[reachable] = np.minimum(window[reachable], -0.7)
            self.terrain_hazard[r0:r1, c0:c1][reachable] = False
        self.surface_reachable_cells = int(reachable.sum())

    def inflated(self, radius_m, occupied_threshold, extra=None):
        obstacles = self.terrain_hazard if extra is None else (self.terrain_hazard | extra)
        return super(TerrainGrid, self).inflated(radius_m, occupied_threshold, obstacles)
