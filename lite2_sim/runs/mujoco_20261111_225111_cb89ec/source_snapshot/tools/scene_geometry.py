"""Engine-independent geometry from the existing Lite2 scene configuration."""

import math


def box(name, position, size, color, rotation=None, collision=True):
    return {"name": name, "type": "box", "position_m": list(position),
            "size_m": list(size), "color": list(color),
            "rotation": rotation, "collision": collision}


def segment_wall(name, start, end, width, height, color):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    if length < 1e-5:
        raise ValueError("A wall segment must have positive length.")
    center = [(start[0] + end[0]) / 2, (start[1] + end[1]) / 2, height / 2]
    return box(name, center, [length, width, height], color,
               [0, 0, 1, math.atan2(dy, dx)])


def sensor_geometry(config):
    width, length = config["field_size_m"]
    shapes = [box("floor", [0, 0, -0.05], [width, length, 0.1], [0.58, 0.6, 0.62])]
    for name, position, size in [
        ("west_wall", [-width / 2, 0, 0.4], [0.12, length, 0.8]),
        ("east_wall", [width / 2, 0, 0.4], [0.12, length, 0.8]),
        ("south_wall", [0, -length / 2, 0.4], [width, 0.12, 0.8]),
        ("north_wall", [0, length / 2, 0.4], [width, 0.12, 0.8]),
    ]:
        shapes.append(box(name, position, size, [0.32, 0.35, 0.38]))
    board = config["calibration_board"]
    shapes.append(box("calibration_board", board["position_m"], board["size_m"], board["color"]))
    for obstacle in config["obstacles"]:
        shapes.append(box(obstacle["name"], obstacle["position_m"], obstacle["size_m"], [0.48, 0.45, 0.4]))

    return shapes


def ruins_geometry(config):
    width, length = config["field_size_m"]
    wall_color = [0.32, 0.35, 0.38]
    rubble_color = [0.48, 0.45, 0.4]
    solids = [box("floor", [0, 0, -0.05], [width, length, 0.1], [0.58, 0.6, 0.62])]
    wall_h = config["outer_wall_height_m"]
    wall_t = config["outer_wall_thickness_m"]
    for name, position, size in [
        ("west_wall", [-width / 2, 0, wall_h / 2], [wall_t, length, wall_h]),
        ("east_wall", [width / 2, 0, wall_h / 2], [wall_t, length, wall_h]),
        ("south_wall", [0, -length / 2, wall_h / 2], [width, wall_t, wall_h]),
        ("north_wall", [0, length / 2, wall_h / 2], [width, wall_t, wall_h]),
    ]:
        solids.append(box(name, position, size, wall_color))

    for ramp in config["ramps"]:
        angle = -math.atan2(ramp["rise_m"], ramp["length_m"])
        solids.append(box(
            ramp["name"], ramp["position_m"],
            [ramp["length_m"], ramp["width_m"], ramp["thickness_m"]],
            [0.62, 0.57, 0.47], [0, 1, 0, angle],
        ))
        # Side rails make ramp edges visible and provide geometry for later clearance checks.
        rise = ramp["rise_m"]
        rail_length = math.hypot(ramp["length_m"], rise)
        rail_angle = -math.atan2(rise, ramp["length_m"])
        for side in (-1, 1):
            rail_position = [
                ramp["position_m"][0],
                ramp["position_m"][1] + side * ramp["width_m"] / 2,
                ramp["position_m"][2] + ramp["rail_height_m"] / 2,
            ]
            solids.append(box(
                ramp["name"] + ("_rail_left" if side > 0 else "_rail_right"),
                rail_position,
                [rail_length, ramp["rail_thickness_m"], ramp["rail_height_m"]],
                [0.38, 0.39, 0.4], [0, 1, 0, rail_angle],
            ))

    # Build the S-corridor as short joined wall segments from a sampled centerline.
    curve = config["s_bend"]
    count = curve["segments"]
    centerline = []
    for index in range(count + 1):
        t = index / count
        y = curve["start_y_m"] + t * (curve["end_y_m"] - curve["start_y_m"])
        x = curve["center_x_m"] + curve["amplitude_m"] * math.sin(2 * math.pi * curve["lobes"] * t)
        centerline.append((x, y))
    left, right = [], []
    for index, point in enumerate(centerline):
        before = centerline[max(0, index - 1)]
        after = centerline[min(count, index + 1)]
        dx, dy = after[0] - before[0], after[1] - before[1]
        norm = math.hypot(dx, dy) or 1
        nx, ny = -dy / norm, dx / norm
        half = curve["clear_width_m"] / 2 + curve["wall_thickness_m"] / 2
        left.append((point[0] + nx * half, point[1] + ny * half))
        right.append((point[0] - nx * half, point[1] - ny * half))
    for side_name, boundary in [("left", left), ("right", right)]:
        for index in range(count):
            solids.append(segment_wall(
                f"s_bend_{side_name}_{index:02d}", boundary[index], boundary[index + 1],
                curve["wall_thickness_m"], curve["wall_height_m"], wall_color,
            ))

    # A U-shaped branch provides a dead end with a clear retreat path.
    fork = config["dead_end"]
    x0, x1 = fork["x_min_m"], fork["x_max_m"]
    y0, y1 = fork["y_entry_m"], fork["y_end_m"]
    solids.extend([
        segment_wall("dead_end_left", [x0, y0], [x0, y1], fork["wall_thickness_m"], fork["wall_height_m"], wall_color),
        segment_wall("dead_end_right", [x1, y0], [x1, y1], fork["wall_thickness_m"], fork["wall_height_m"], wall_color),
        segment_wall("dead_end_cap", [x0, y1], [x1, y1], fork["wall_thickness_m"], fork["wall_height_m"], wall_color),
    ])

    # Five deterministic rubble patches use the same boxes for rendering and collision.
    for pile_index, pile in enumerate(config["rubble_piles"], start=1):
        n = pile["stone_count"]
        radius_x, radius_y = pile["footprint_m"]
        for stone_index in range(n):
            angle = stone_index * 2.399963229728653
            radial = math.sqrt((stone_index + 0.5) / n)
            sx = radius_x * 0.5 * radial * math.cos(angle)
            sy = radius_y * 0.5 * radial * math.sin(angle)
            layer = 1 if stone_index % 5 == 0 and stone_index > 0 else 0
            size = [0.17 + 0.04 * (stone_index % 3), 0.14 + 0.035 * (stone_index % 4), 0.12 + 0.04 * ((stone_index + 1) % 3)]
            if layer:
                size[2] *= 0.85
            position = [pile["center_m"][0] + sx, pile["center_m"][1] + sy, size[2] / 2 + layer * 0.13]
            solids.append(box(
                f"rubble_{pile_index}_stone_{stone_index + 1:02d}", position, size,
                rubble_color, [0, 0, 1, angle * 0.35],
            ))

    tunnels = config["tunnels"]
    narrow = tunnels["narrow_gate"]
    nx, ny = narrow["center_m"]
    nt, nh, nw = narrow["wall_thickness_m"], narrow["height_m"], narrow["opening_width_m"]
    solids.extend([
        box("narrow_tunnel_left", [nx, ny - nw / 2 - nt / 2, nh / 2], [narrow["length_m"], nt, nh], wall_color),
        box("narrow_tunnel_right", [nx, ny + nw / 2 + nt / 2, nh / 2], [narrow["length_m"], nt, nh], wall_color),
        box("narrow_tunnel_header", [nx, ny, nh + nt / 2], [narrow["length_m"], nw + 2 * nt, nt], wall_color),
    ])

    pipe = tunnels["circular_pipe"]
    px, py = pipe["center_m"]
    radial_mid = (pipe["inner_radius_m"] + pipe["outer_radius_m"]) / 2
    radial_width = pipe["outer_radius_m"] - pipe["inner_radius_m"]
    segment_count = pipe["segments"]
    tangential = 2 * radial_mid * math.sin(math.pi / segment_count) * 1.08
    for index in range(segment_count):
        theta = 2 * math.pi * index / segment_count
        position = [px, py + radial_mid * math.sin(theta), pipe["inner_radius_m"] + radial_mid * math.cos(theta)]
        solids.append(box(
            f"circular_pipe_segment_{index:02d}", position,
            [pipe["length_m"], tangential, radial_width], [0.42, 0.44, 0.46],
            [1, 0, 0, -theta],
        ))

    culvert = tunnels["rectangular_culvert"]
    cx, cy = culvert["center_m"]
    ct, cw, ch = culvert["wall_thickness_m"], culvert["inner_width_m"], culvert["inner_height_m"]
    solids.extend([
        box("culvert_left", [cx, cy - cw / 2 - ct / 2, ch / 2], [culvert["length_m"], ct, ch], wall_color),
        box("culvert_right", [cx, cy + cw / 2 + ct / 2, ch / 2], [culvert["length_m"], ct, ch], wall_color),
        box("culvert_roof", [cx, cy, ch + ct / 2], [culvert["length_m"], cw + 2 * ct, ct], wall_color),
    ])

    for marker in config["markers"]:
        color = [0.12, 0.65, 0.2] if marker["kind"] == "start" else [0.2, 0.3, 0.85]
        mx, my = marker["position_m"]
        parts = [
            ([mx - 0.10, my, 0.003], [0.055, 0.36, 0.006]),
            ([mx + 0.10, my, 0.003], [0.055, 0.36, 0.006]),
            ([mx, my, 0.003], [0.24, 0.055, 0.006]),
        ]
        for index, (position, size) in enumerate(parts):
            solids.append(box(marker["name"] + "_" + str(index), position,
                              size, color, collision=False))
    for target in config["target_placeholders"]:
        solids.append({"name": target["name"], "type": "cylinder",
                       "position_m": list(target["position_m"]),
                       "size_m": [0.12, 0.38], "color": [0.9, 0.58, 0.1],
                       "rotation": None, "collision": True})
    return solids


def build_geometry(config, scene_id):
    if scene_id == "sensor_lab":
        return sensor_geometry(config)
    if scene_id == "ruins_reference":
        return ruins_geometry(config)
    raise ValueError("Unknown scene: " + scene_id)
