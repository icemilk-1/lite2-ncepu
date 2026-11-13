"""Small 2-D scan-to-map SLAM and A* navigation loop for the Webots course."""

import csv
import heapq
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class OccupancyGrid:
    def __init__(self, field_size, resolution):
        self.resolution = float(resolution)
        self.x_min, self.y_min = -field_size[0] / 2, -field_size[1] / 2
        self.width = int(round(field_size[0] / self.resolution))
        self.height = int(round(field_size[1] / self.resolution))
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.scan_count = 0
        self.hit_count = 0

    def world_to_cell(self, x, y):
        column = int(math.floor((x - self.x_min) / self.resolution))
        row = int(math.floor((y - self.y_min) / self.resolution))
        if 0 <= column < self.width and 0 <= row < self.height:
            return column, row
        return None

    def cell_to_world(self, column, row):
        return (
            self.x_min + (column + 0.5) * self.resolution,
            self.y_min + (row + 0.5) * self.resolution,
        )

    def ray_cells(self, x0, y0, x1, y1):
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        error = dx - dy
        cells = []
        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            twice_error = 2 * error
            if twice_error > -dy:
                error -= dy
                x0 += sx
            if twice_error < dx:
                error += dx
                y0 += sy
        return cells

    def integrate(self, points, pose, extrinsic, max_range, min_range, stride=1):
        x, y, yaw = pose
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        sensor_x = x + cos_yaw * extrinsic[0] - sin_yaw * extrinsic[1]
        sensor_y = y + sin_yaw * extrinsic[0] + cos_yaw * extrinsic[1]
        sensor_cell = self.world_to_cell(sensor_x, sensor_y)
        if sensor_cell is None:
            return 0
        integrated = 0
        hits = 0
        for local_x, local_y, measured_hit in points[::stride]:
            distance = math.hypot(local_x, local_y)
            if not math.isfinite(distance) or distance < min_range:
                continue
            hit = measured_hit and distance < max_range - 0.04
            distance = min(distance, max_range)
            angle = yaw + math.atan2(local_y, local_x)
            end_x = sensor_x + distance * math.cos(angle)
            end_y = sensor_y + distance * math.sin(angle)
            end_cell = self.world_to_cell(end_x, end_y)
            if end_cell is None:
                # Clip rays to the map rectangle before rasterizing.
                scale = min(
                    (self.x_min + self.width * self.resolution - sensor_x) / (end_x - sensor_x) if end_x > self.x_min + self.width * self.resolution else math.inf,
                    (self.x_min - sensor_x) / (end_x - sensor_x) if end_x < self.x_min else math.inf,
                    (self.y_min + self.height * self.resolution - sensor_y) / (end_y - sensor_y) if end_y > self.y_min + self.height * self.resolution else math.inf,
                    (self.y_min - sensor_y) / (end_y - sensor_y) if end_y < self.y_min else math.inf,
                    1.0,
                )
                if not math.isfinite(scale) or scale <= 0:
                    continue
                scale = max(0.0, min(1.0, scale * (1.0 - 1e-6)))
                end_x = sensor_x + distance * scale * math.cos(angle)
                end_y = sensor_y + distance * scale * math.sin(angle)
                end_cell = self.world_to_cell(end_x, end_y)
                if end_cell is None:
                    continue
                hit = False

            ray = self.ray_cells(sensor_cell[0], sensor_cell[1], end_cell[0], end_cell[1])
            free_cells = ray[:-1] if hit else ray
            if free_cells:
                columns, rows = zip(*free_cells)
                columns, rows = np.asarray(columns), np.asarray(rows)
                inside = (columns >= 0) & (columns < self.width) & (rows >= 0) & (rows < self.height)
                self.log_odds[rows[inside], columns[inside]] = np.maximum(
                    -4.0, self.log_odds[rows[inside], columns[inside]] - 0.35,
                )
            if hit:
                column, row = end_cell
                self.log_odds[row, column] = min(4.0, self.log_odds[row, column] + 0.9)
                hits += 1
            integrated += 1
        self.scan_count += 1
        self.hit_count += hits
        return integrated

    def match_pose(self, prior, points, extrinsic, max_range, search_xy, search_yaw):
        if self.scan_count == 0:
            return prior
        useful = [(x, y) for x, y, hit in points if hit and math.isfinite(x) and math.isfinite(y) and 0.3 < math.hypot(x, y) < max_range - 0.05]
        if len(useful) > 100:
            useful = useful[::max(1, len(useful) // 100)]
        if len(useful) < 8:
            return prior
        local = np.asarray(useful, dtype=np.float32)
        best_pose = prior
        best_score = -math.inf
        xy_step = self.resolution * 0.5
        yaw_step = 0.04
        offsets_xy = np.arange(-search_xy, search_xy + xy_step * 0.5, xy_step)
        offsets_yaw = np.arange(-search_yaw, search_yaw + yaw_step * 0.5, yaw_step)
        for dyaw in offsets_yaw:
            yaw = wrap_angle(prior[2] + float(dyaw))
            c, s = math.cos(yaw), math.sin(yaw)
            local_x = local[:, 0]
            local_y = local[:, 1]
            for dx in offsets_xy:
                for dy in offsets_xy:
                    x = prior[0] + float(dx)
                    y = prior[1] + float(dy)
                    sx = x + c * extrinsic[0] - s * extrinsic[1]
                    sy = y + s * extrinsic[0] + c * extrinsic[1]
                    wx = sx + c * local_x - s * local_y
                    wy = sy + s * local_x + c * local_y
                    columns = np.floor((wx - self.x_min) / self.resolution).astype(np.int32)
                    rows = np.floor((wy - self.y_min) / self.resolution).astype(np.int32)
                    inside = (columns >= 0) & (columns < self.width) & (rows >= 0) & (rows < self.height)
                    values = np.zeros(len(local_x), dtype=np.float32)
                    values[inside] = self.log_odds[rows[inside], columns[inside]]
                    alignment = float(np.mean(np.clip(values, -1.0, 2.5)))
                    prior_cost = 0.08 * ((dx / max(search_xy, 1e-4)) ** 2 + (dy / max(search_xy, 1e-4)) ** 2)
                    prior_cost += 0.08 * (dyaw / max(search_yaw, 1e-4)) ** 2
                    score = alignment - prior_cost
                    if score > best_score:
                        best_pose, best_score = (x, y, yaw), score
        return best_pose

    def inflated(self, radius_m, occupied_threshold):
        occupied = self.log_odds >= occupied_threshold
        radius = int(math.ceil(radius_m / self.resolution))
        result = occupied.copy()
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx * dx + dy * dy > radius * radius:
                    continue
                source_x0, source_x1 = max(0, -dx), min(self.width, self.width - dx)
                source_y0, source_y1 = max(0, -dy), min(self.height, self.height - dy)
                target_x0, target_x1 = source_x0 + dx, source_x1 + dx
                target_y0, target_y1 = source_y0 + dy, source_y1 + dy
                result[target_y0:target_y1, target_x0:target_x1] |= occupied[source_y0:source_y1, source_x0:source_x1]
        edge = max(1, radius)
        result[:edge, :] = True
        result[-edge:, :] = True
        result[:, :edge] = True
        result[:, -edge:] = True
        return result

    def plan(self, start, goal, radius_m, occupied_threshold, unknown_cost, preferred_clearance_m=0.0):
        blocked = self.inflated(radius_m, occupied_threshold)
        preferred_blocked = self.inflated(radius_m + preferred_clearance_m, occupied_threshold)
        start_cell = self.world_to_cell(start[0], start[1])
        goal_cell = self.world_to_cell(goal[0], goal[1])
        if start_cell is None or goal_cell is None:
            return []
        start_cell = self.nearest_open(start_cell, blocked, 8)
        goal_cell = self.nearest_open(goal_cell, blocked, 12)
        if start_cell is None or goal_cell is None:
            return []

        def heuristic(cell):
            return math.hypot(goal_cell[0] - cell[0], goal_cell[1] - cell[1])

        frontier = [(heuristic(start_cell), 0.0, start_cell)]
        came_from = {}
        cost_so_far = {start_cell: 0.0}
        neighbors = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                     (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
                     (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]
        while frontier:
            _, cost, current = heapq.heappop(frontier)
            if current == goal_cell:
                break
            if cost > cost_so_far.get(current, math.inf):
                continue
            for dx, dy, distance_cost in neighbors:
                nxt = (current[0] + dx, current[1] + dy)
                if not (0 <= nxt[0] < self.width and 0 <= nxt[1] < self.height) or blocked[nxt[1], nxt[0]]:
                    continue
                if dx and dy and (blocked[current[1], nxt[0]] or blocked[nxt[1], current[0]]):
                    continue
                evidence = self.log_odds[nxt[1], nxt[0]]
                cell_cost = 1.0 if evidence < -0.18 else unknown_cost
                if preferred_blocked[nxt[1], nxt[0]]:
                    cell_cost += 2.0
                next_cost = cost + distance_cost * cell_cost
                if next_cost < cost_so_far.get(nxt, math.inf):
                    cost_so_far[nxt] = next_cost
                    came_from[nxt] = current
                    heapq.heappush(frontier, (next_cost + heuristic(nxt), next_cost, nxt))
        else:
            return []

        cells = [goal_cell]
        while cells[-1] != start_cell:
            cells.append(came_from[cells[-1]])
        cells.reverse()
        # Preserve the preferred margin while smoothing. Narrow passages keep
        # their original A* cells when the extra margin cannot be attained.
        cells = self.smooth(cells, preferred_blocked)
        return [self.cell_to_world(column, row) for column, row in cells]

    @staticmethod
    def nearest_open(cell, blocked, max_radius):
        if not blocked[cell[1], cell[0]]:
            return cell
        for radius in range(1, max_radius + 1):
            candidates = []
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    x, y = cell[0] + dx, cell[1] + dy
                    if 0 <= x < blocked.shape[1] and 0 <= y < blocked.shape[0] and not blocked[y, x]:
                        candidates.append((dx * dx + dy * dy, (x, y)))
            if candidates:
                return min(candidates)[1]
        return None

    @staticmethod
    def smooth(cells, blocked):
        if len(cells) < 3:
            return cells

        def clear(a, b):
            cells_on_line = OccupancyGrid.line_cells(a, b)
            return all(not blocked[y, x] for x, y in cells_on_line)

        result = [cells[0]]
        current = 0
        while current < len(cells) - 1:
            candidate = len(cells) - 1
            while candidate > current + 1 and not clear(cells[current], cells[candidate]):
                candidate -= 1
            result.append(cells[candidate])
            current = candidate
        return result

    @staticmethod
    def line_cells(start, end):
        x0, y0 = start
        x1, y1 = end
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        error = dx - dy
        points = []
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                return points
            twice = 2 * error
            if twice > -dy and twice < dx:
                points.append((x0 + sx, y0))
                points.append((x0, y0 + sy))
            if twice > -dy:
                error -= dy
                x0 += sx
            if twice < dx:
                error += dx
                y0 += sy

    def segment_blocked(self, start_xy, end_xy, blocked):
        start_cell = self.world_to_cell(start_xy[0], start_xy[1])
        end_cell = self.world_to_cell(end_xy[0], end_xy[1])
        if start_cell is None or end_cell is None:
            return True
        cells = self.line_cells(start_cell, end_cell)
        # Ignore the starting cell so a robot already touching an observed boundary can move away from it.
        return any(blocked[row, column] for column, row in cells[1:])

    def save(self, output, path, start, goal, radius_m, occupied_threshold, trajectory=None):
        output = Path(output)
        occupied = self.log_odds >= occupied_threshold
        free = self.log_odds <= -0.18
        image = np.full((self.height, self.width), 205, dtype=np.uint8)
        image[free] = 250
        image[occupied] = 25
        pgm = np.flipud(image)
        with (output / "slam_map.pgm").open("wb") as stream:
            stream.write(f"P5\n{self.width} {self.height}\n255\n".encode("ascii"))
            stream.write(pgm.tobytes())
        (output / "slam_map.yaml").write_text(
            "image: slam_map.pgm\n"
            f"resolution: {self.resolution}\n"
            f"origin: [{self.x_min}, {self.y_min}, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n",
            encoding="utf-8",
        )
        rgb = np.repeat(pgm[:, :, None], 3, axis=2)
        canvas = Image.fromarray(rgb, mode="RGB")
        draw = ImageDraw.Draw(canvas)

        def pixel(point):
            cell = self.world_to_cell(point[0], point[1])
            if cell is None:
                return None
            return cell[0], self.height - 1 - cell[1]

        path_pixels = [pixel(point) for point in path]
        path_pixels = [point for point in path_pixels if point is not None]
        if len(path_pixels) > 1:
            draw.line(path_pixels, fill=(20, 150, 35), width=max(1, int(0.08 / self.resolution)))
        trajectory_pixels = [pixel(point) for point in trajectory or []]
        trajectory_pixels = [point for point in trajectory_pixels if point is not None]
        if len(trajectory_pixels) > 1:
            draw.line(trajectory_pixels, fill=(35, 90, 220), width=max(1, int(0.06 / self.resolution)))
        for point, color in [(start, (35, 100, 230)), (goal, (220, 35, 35))]:
            p = pixel(point)
            if p is not None:
                radius = max(2, int(radius_m / self.resolution))
                draw.ellipse((p[0] - radius, p[1] - radius, p[0] + radius, p[1] + radius), fill=color)
        canvas = canvas.resize((self.width * 5, self.height * 5), Image.Resampling.NEAREST)
        canvas.save(output / "slam_map.png")
        metadata = {
            "frame_id": "map",
            "resolution_m": self.resolution,
            "width_cells": self.width,
            "height_cells": self.height,
            "origin_xy_m": [self.x_min, self.y_min],
            "integrated_scans": self.scan_count,
            "occupied_cells": int(occupied.sum()),
            "free_cells": int(free.sum()),
            "unknown_cells": int((~occupied & ~free).sum()),
        }
        (output / "slam_map.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        return metadata


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


def _save_path(path, output, start_pose, goal):
    (Path(output) / "planned_path.json").write_text(
        json.dumps({
            "frame_id": "map", "planner": "8-connected A* with line-of-sight smoothing",
            "planning_snapshot": "initial accumulated SLAM map",
            "start_xy_m": list(start_pose), "goal_xy_m": list(goal), "poses_xy_m": path,
        }, indent=2),
        encoding="utf-8",
    )


def _save_current_path(path, output, pose, goal, sim_time_s, history):
    record = {
        "sim_time_s": sim_time_s,
        "from_xy_m": list(pose[:2]),
        "poses_xy_m": path,
        "path_length_m": sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:])),
    }
    history.append(record)
    (Path(output) / "current_path.json").write_text(
        json.dumps({"frame_id": "map", "planner": "8-connected A*", "current_replan": record, "history": history}, indent=2),
        encoding="utf-8",
    )


def run_navigation(robot, config, output, timestep):
    settings = config["navigation"]
    planning_clearance = settings.get("planning_clearance_m", 0.0)
    lidar_settings = config["lidar"]
    output = Path(output)
    lidar = robot.getDevice("front_lidar")
    lidar.enable(timestep)
    lidar.enablePointCloud()
    lidar.setFrequency(lidar_settings["frequency_hz"])
    fixture = robot.getFromDef("ROBOT_FIXTURE")
    if fixture is None:
        raise RuntimeError("Webots world is missing DEF ROBOT_FIXTURE.")
    translation_field = fixture.getField("translation")
    rotation_field = fixture.getField("rotation")
    if translation_field is None or rotation_field is None:
        raise RuntimeError("The robot fixture has no mutable pose fields.")

    grid = OccupancyGrid(config["field_size_m"], settings["grid_resolution_m"])
    true_pose = [config["robot_position_m"][0], config["robot_position_m"][1], 0.0]
    odom_pose = true_pose.copy()
    slam_pose = true_pose.copy()
    goal = tuple(settings["goal_xy_m"])
    extrinsic = lidar_settings["mount_xy_m"]
    sensor_height_world = config["robot_position_m"][2] + lidar_settings["mount_xyz_m"][2]
    dt = timestep / 1000.0
    random = np.random.default_rng(settings["random_seed"])
    scan_period = settings["scan_mapping_period_s"]
    next_scan_time = robot.getTime()
    last_replan_time = -math.inf
    path = []
    trace = []
    mapping_start = robot.getTime()
    navigation_start = None
    initial_mapping_done = False
    status = "mapping"
    travelled = 0.0
    last_position = tuple(true_pose[:2])
    last_progress_time = robot.getTime()
    correction_count = 0
    current_command = (0.0, 0.0)
    last_scan_points = []
    scan_metadata = None
    last_status_report = mapping_start
    initial_path = []
    replan_history = []
    collision_map = None
    collision_stop_count = 0

    while robot.step(timestep) != -1:
        now = robot.getTime()
        if now >= next_scan_time:
            capture_path = output / "lidar_raw_first.npz" if grid.scan_count == 0 else None
            last_scan_points, scan_metadata = _scan_data(lidar, lidar_settings, sensor_height_world, capture_path)
            if grid.scan_count == 0 and scan_metadata["scan_ready"]:
                (output / "lidar_scan.json").write_text(json.dumps(scan_metadata, indent=2, allow_nan=False), encoding="utf-8")
            if scan_metadata["scan_ready"]:
                if grid.scan_count:
                    matched = grid.match_pose(
                        odom_pose, last_scan_points, extrinsic, lidar_settings["max_range_m"],
                        settings["scan_match_search_xy_m"], settings["scan_match_search_yaw_rad"],
                    )
                    if math.hypot(matched[0] - odom_pose[0], matched[1] - odom_pose[1]) > 0.005 or abs(wrap_angle(matched[2] - odom_pose[2])) > 0.01:
                        correction_count += 1
                    slam_pose = list(matched)
                else:
                    slam_pose = odom_pose.copy()
                grid.integrate(
                    last_scan_points, slam_pose, extrinsic,
                    lidar_settings["max_range_m"], lidar_settings["min_range_m"],
                    settings["scan_stride"],
                )
                collision_map = grid.inflated(
                    settings["robot_radius_m"], settings["occupied_log_odds_threshold"],
                )
                trace.append((now, *true_pose, *odom_pose, *slam_pose, *current_command))
            next_scan_time = now + scan_period

        if not initial_mapping_done and grid.scan_count >= 3 and now - mapping_start >= settings["initial_mapping_duration_s"]:
            path = grid.plan(
                slam_pose, goal, settings["robot_radius_m"],
                settings["occupied_log_odds_threshold"], settings["unknown_cell_cost"],
                planning_clearance,
            )
            last_replan_time = now
            navigation_start = now
            initial_mapping_done = True
            status = "navigating" if path else "no_path"
            initial_path = path.copy()
            _save_path(initial_path, output, slam_pose[:2], goal)
            _save_current_path(path, output, slam_pose, goal, now, replan_history)
            print(f"SLAM_MAP_READY scans={grid.scan_count} cells={grid.width}x{grid.height} path_points={len(path)}", flush=True)

        if initial_mapping_done and status == "navigating":
            distance_to_goal = math.hypot(goal[0] - slam_pose[0], goal[1] - slam_pose[1])
            if distance_to_goal <= settings["goal_tolerance_m"]:
                status = "goal_reached"
                current_command = (0.0, 0.0)
            elif now - navigation_start >= settings["navigation_timeout_s"]:
                status = "timeout"
                current_command = (0.0, 0.0)
            elif now - last_replan_time >= settings["replan_period_s"] or not path:
                path = grid.plan(
                    slam_pose, goal, settings["robot_radius_m"],
                    settings["occupied_log_odds_threshold"], settings["unknown_cell_cost"],
                    planning_clearance,
                )
                last_replan_time = now
                _save_current_path(path, output, slam_pose, goal, now, replan_history)
                if not path:
                    status = "no_path"
                    current_command = (0.0, 0.0)
            if status == "navigating" and path:
                point_index = min(range(len(path)), key=lambda idx: math.hypot(path[idx][0] - slam_pose[0], path[idx][1] - slam_pose[1]))
                target_index = point_index
                while target_index < len(path) - 1 and math.hypot(path[target_index][0] - slam_pose[0], path[target_index][1] - slam_pose[1]) < settings["lookahead_m"]:
                    target_index += 1
                target = path[target_index]
                heading_error = wrap_angle(math.atan2(target[1] - slam_pose[1], target[0] - slam_pose[0]) - slam_pose[2])
                front_ranges = [
                    math.hypot(px, py) for px, py, hit in last_scan_points
                    if hit and px > 0 and abs(math.atan2(py, px)) < settings["front_stop_angle_rad"]
                    and math.hypot(px, py) > lidar_settings["min_range_m"]
                ]
                obstacle_ahead = front_ranges and min(front_ranges) < settings["front_stop_distance_m"]
                forward_speed = settings["max_linear_speed_m_s"] * max(0.0, math.cos(heading_error))
                if abs(heading_error) > 1.15 or obstacle_ahead:
                    forward_speed = 0.0
                yaw_speed = float(np.clip(2.0 * heading_error, -settings["max_yaw_speed_rad_s"], settings["max_yaw_speed_rad_s"]))
                current_command = (forward_speed, yaw_speed)

        if current_command[0] or current_command[1]:
            v, w = current_command
            old_xy = tuple(true_pose[:2])
            mid_yaw = true_pose[2] + w * dt * 0.5
            next_x = true_pose[0] + v * math.cos(mid_yaw) * dt
            next_y = true_pose[1] + v * math.sin(mid_yaw) * dt
            if collision_map is not None and grid.segment_blocked(
                true_pose[:2], (next_x, next_y), collision_map,
            ):
                collision_stop_count += 1
                v = 0.0
                current_command = (0.0, w)
            else:
                true_pose[0], true_pose[1] = next_x, next_y
            true_pose[2] = wrap_angle(true_pose[2] + w * dt)
            traveled_step = math.hypot(true_pose[0] - old_xy[0], true_pose[1] - old_xy[1])
            travelled += traveled_step
            odom_v = v * (1.0 + random.normal(0.0, settings["odom_linear_noise_fraction"]))
            odom_w = w + random.normal(0.0, settings["odom_yaw_noise_rad_s"])
            odom_mid_yaw = odom_pose[2] + odom_w * dt * 0.5
            odom_pose[0] += odom_v * math.cos(odom_mid_yaw) * dt
            odom_pose[1] += odom_v * math.sin(odom_mid_yaw) * dt
            odom_pose[2] = wrap_angle(odom_pose[2] + odom_w * dt)
            if math.hypot(true_pose[0] - last_position[0], true_pose[1] - last_position[1]) >= 0.08:
                last_position = tuple(true_pose[:2])
                last_progress_time = now
        if initial_mapping_done and status == "navigating" and now - last_progress_time > settings["stuck_timeout_s"]:
            status = "stalled"
            current_command = (0.0, 0.0)

        translation_field.setSFVec3f([true_pose[0], true_pose[1], config["robot_position_m"][2]])
        rotation_field.setSFRotation([0.0, 0.0, 1.0, true_pose[2]])

        if initial_mapping_done and status != "navigating":
            break
        if now - last_status_report >= 5.0:
            grid.save(
                output, initial_path, (config["robot_position_m"][0], config["robot_position_m"][1]), goal,
                settings["robot_radius_m"], settings["occupied_log_odds_threshold"],
                [(record[1], record[2]) for record in trace],
            )
            print(
                f"NAV_PROGRESS t={now - mapping_start:.1f}s status={status} "
                f"pose=({slam_pose[0]:.2f},{slam_pose[1]:.2f}) scans={grid.scan_count}",
                flush=True,
            )
            last_status_report = now

    map_metadata = grid.save(
        output, initial_path, (config["robot_position_m"][0], config["robot_position_m"][1]), goal,
        settings["robot_radius_m"], settings["occupied_log_odds_threshold"],
        [(record[1], record[2]) for record in trace],
    )
    _save_current_path(path, output, slam_pose, goal, robot.getTime(), replan_history)
    (output / "replan_history.json").write_text(json.dumps(replan_history, indent=2), encoding="utf-8")
    with (output / "slam_trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["sim_time_s", "truth_x_m_eval", "truth_y_m_eval", "truth_yaw_rad_eval", "odom_x_m", "odom_y_m", "odom_yaw_rad", "slam_x_m", "slam_y_m", "slam_yaw_rad", "cmd_v_m_s", "cmd_w_rad_s"])
        writer.writerows(trace)
    initial_route_length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(initial_path, initial_path[1:]))
    remaining_route_length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))
    summary = {
        "result": status,
        "scene_id": config.get("scene_id", "ruins_reference"),
        "mapping": "2-D occupancy grid with odometry-prior local scan-to-map pose correction",
        "loop_closure": False,
        "sensor": {"model": lidar_settings["model_status"], "horizontal_resolution": lidar_settings["horizontal_resolution"], "field_of_view_rad": lidar_settings["field_of_view_rad"], "max_range_m": lidar_settings["max_range_m"]},
        "planning": {"planner": "8-connected A* with obstacle inflation and line-of-sight smoothing", "robot_radius_m": settings["robot_radius_m"], "planning_clearance_m": settings.get("planning_clearance_m", 0.0), "goal_xy_m": list(goal), "initial_path_points": len(initial_path), "initial_planned_path_length_m": initial_route_length, "final_remaining_path_length_m": remaining_route_length, "replans": max(0, len(replan_history) - 1)},
        "simulation_model": "kinematic planar base; manufacturer joint pose held fixed; no gait or terrain contact simulation",
        "evaluation_only": {"final_truth_xy_m": true_pose[:2], "goal_error_m": math.hypot(goal[0] - true_pose[0], goal[1] - true_pose[1]), "distance_travelled_m": travelled, "scan_match_corrections": correction_count, "collision_stop_events": collision_stop_count},
        "map": map_metadata,
    }
    (output / "navigation_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print("NAVIGATION_FINISHED " + json.dumps({"result": status, "distance_m": round(travelled, 2), "scans": grid.scan_count, "map": str(output / "slam_map.png")}, ensure_ascii=False), flush=True)
    return summary
