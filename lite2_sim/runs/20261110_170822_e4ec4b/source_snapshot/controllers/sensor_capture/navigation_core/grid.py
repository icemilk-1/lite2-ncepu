"""Portable NumPy occupancy mapping and geometric helpers."""

import heapq
import math

import numpy as np


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
        free_mask = np.zeros((self.height, self.width), dtype=bool)
        hit_mask = np.zeros((self.height, self.width), dtype=bool)
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
                free_mask[rows[inside], columns[inside]] = True
            if hit:
                column, row = end_cell
                hit_mask[row, column] = True
                hits += 1
            integrated += 1
        # A scan is one observation, even when adjacent beams cross the same
        # cell. Count each cell once and never clear an occupied endpoint with
        # a neighboring beam from this same scan.
        free_mask &= ~hit_mask
        self.log_odds[free_mask] = np.maximum(-4.0, self.log_odds[free_mask] - 0.35)
        self.log_odds[hit_mask] = np.minimum(4.0, self.log_odds[hit_mask] + 0.9)
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

    def plan_result(self, start, goal, radius_m, occupied_threshold, unknown_cost=2.8, preferred_clearance_m=0.0):
        from navigation_core.planner import plan_grid
        return plan_grid(self, start, goal, radius_m, occupied_threshold, unknown_cost, preferred_clearance_m)

    def plan(self, start, goal, radius_m, occupied_threshold, unknown_cost, preferred_clearance_m=0.0):
        return self.plan_result(start, goal, radius_m, occupied_threshold, unknown_cost, preferred_clearance_m).path

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
