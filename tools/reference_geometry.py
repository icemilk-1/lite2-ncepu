"""Arena primitives reconstructed from the user's November 11 reference image."""

import math

import numpy as np
import trimesh

if __package__:
    from .scene_geometry import box, segment_wall
else:
    from scene_geometry import box, segment_wall


def mesh_shape(name, position, vertices, faces, color, collision=True, rotation=None):
    return {"name": name, "type": "mesh", "position_m": list(position),
            "size_m": [], "vertices_m": np.asarray(vertices).tolist(),
            "faces": np.asarray(faces, dtype=int).tolist(), "color": list(color),
            "collision": collision, "rotation": rotation}


def sampled_path(points, smooth):
    points = np.asarray(points, dtype=float)
    if not smooth:
        return points
    result = []
    for index in range(len(points) - 1):
        p0, p1 = points[max(0, index - 1)], points[index]
        p2, p3 = points[index + 1], points[min(len(points) - 1, index + 2)]
        steps = max(2, math.ceil(np.linalg.norm(p2 - p1) / 0.12))
        for t in np.linspace(0, 1, steps, endpoint=False):
            result.append(0.5 * ((2 * p1) + (-p0 + p2) * t +
                                (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t +
                                (-p0 + 3 * p1 - 3 * p2 + p3) * t * t * t))
    result.append(points[-1])
    return np.asarray(result)


def brick_wall(name, points, style, smooth=False, height=None):
    path = sampled_path(points, smooth)
    height = style["height_m"] if height is None else height
    thickness, color = style["thickness_m"], np.asarray(style["color"])
    shapes = []
    for index, (start, end) in enumerate(zip(path[:-1], path[1:])):
        shapes.append(segment_wall(name + "_collision_" + str(index), start, end,
                                   thickness, height, (color * 0.73).tolist()))
    lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
    cumulative = np.concatenate(([0], np.cumsum(lengths)))
    brick_length = style["brick_length_m"]
    rows = max(1, round(height / style["brick_height_m"]))
    row_height = height / rows
    gap = style["joint_gap_m"]
    for row in range(rows):
        start_distance = 0.0
        index = 0
        first_length = brick_length / 2 if row % 2 else brick_length
        while start_distance < cumulative[-1] - 1e-6:
            end_distance = min(cumulative[-1], start_distance + (first_length if index == 0 else brick_length))
            center_distance = (start_distance + end_distance) / 2
            segment = min(len(lengths) - 1, int(np.searchsorted(cumulative, center_distance, side="right") - 1))
            direction = (path[segment + 1] - path[segment]) / lengths[segment]
            center = path[segment] + direction * (center_distance - cumulative[segment])
            variation = 0.95 + 0.025 * ((index * 7 + row * 3) % 5)
            shapes.append(box(
                f"{name}_brick_{row}_{index}", [*center, (row + 0.5) * row_height],
                [max(0.003, end_distance - start_distance - gap), thickness + 0.004, row_height - gap],
                (color * variation).tolist(), [0, 0, 1, math.atan2(direction[1], direction[0])], collision=False))
            start_distance = end_distance
            index += 1
    return shapes


def annular_segments(name, center, inner_radius, outer_radius, length, count, axis, color, collision):
    shapes = []
    faces = [[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
             [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5],
             [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]]
    for index in range(count):
        a, b = index * 2 * math.pi / count, (index + 1) * 2 * math.pi / count
        cross_section = [(inner_radius, a), (outer_radius, a), (outer_radius, b), (inner_radius, b)]
        vertices = []
        for along in (-length / 2, length / 2):
            for radius, angle in cross_section:
                u, v = radius * math.cos(angle), radius * math.sin(angle)
                vertices.append([along, u, v] if axis == "x" else [u, v, along])
        shapes.append(mesh_shape(name + "_" + str(index), center, vertices, faces, color, collision))
    return shapes


def reference_geometry(config):
    width, length = config["field_size_m"]
    style = config["wall_style"]
    shapes = [box("floor", [0, 0, -0.05], [width, length, 0.1], [0.59, 0.59, 0.57]),
              box("concrete_apron", [0, 0, -0.14], [width + 2, length + 2, 0.1], [0.48, 0.49, 0.47], collision=False)]
    boundary = [[-width / 2, -length / 2], [width / 2, -length / 2],
                [width / 2, length / 2], [-width / 2, length / 2], [-width / 2, -length / 2]]
    shapes.extend(brick_wall("perimeter", boundary, style))
    # Stable collision names also support the existing known-distance sensor check.
    for name, start, end in [("west_wall", boundary[3], boundary[0]),
                             ("east_wall", boundary[1], boundary[2]),
                             ("south_wall", boundary[0], boundary[1]),
                             ("north_wall", boundary[2], boundary[3])]:
        shapes.append(segment_wall(name, start, end, style["thickness_m"], style["height_m"],
                                   (np.asarray(style["color"]) * 0.73).tolist()))
    # Concrete slab joints are visual geometry at floor level.
    for index, x in enumerate(np.arange(-4.0, 6.0, 2.0)):
        shapes.append(box("floor_joint_x_" + str(index), [x, 0, 0.001], [0.007, length, 0.002], [0.47, 0.47, 0.45], collision=False))
    for index, y in enumerate(np.arange(-3.0, 5.0, 2.0)):
        shapes.append(box("floor_joint_y_" + str(index), [0, y, 0.001], [width, 0.007, 0.002], [0.47, 0.47, 0.45], collision=False))
    for path in config["wall_paths"]:
        shapes.extend(brick_wall(path["name"], path["points_m"], style, path["smooth"]))
    ramp = config.get("upper_left_ramp")
    if ramp:
        faces = [[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
                 [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5],
                 [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]]
        for section in ramp["sections"]:
            x0, x1, y0, y1 = section["bounds_m"]
            corners = section.get("corners_xy_m", [[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
            vertices = [[x, y, -0.02] for x, y in corners]
            vertices.extend([[x, y, height] for (x, y), height in zip(corners, section["corner_heights_m"])])
            shapes.append(mesh_shape(section["name"], [0, 0, 0], vertices, faces, [0.56, 0.55, 0.52]))

    for index, pier in enumerate(config["capped_piers"]):
        x, y = pier["center_m"]
        sx, sy, height = pier["size_m"]
        shapes.append(box(f"pier_{index}", [x, y, height / 2], [sx, sy, height], style["color"]))
        shapes.append(box(f"pier_cap_{index}", [x, y, height + 0.045], [sx + 0.07, sy + 0.07, 0.09], [0.62, 0.62, 0.59]))

    gate = config.get("partition_culvert")
    if gate:
        x, y = gate["center_m"]
        wall_style = dict(style, thickness_m=gate["wall_thickness_m"])
        for side in (-1, 1):
            wall_y = y + side * (gate["clear_width_m"] + gate["wall_thickness_m"]) / 2
            shapes.extend(brick_wall("partition_culvert_side_" + str(side),
                                     [[x - gate["length_m"] / 2, wall_y], [x + gate["length_m"] / 2, wall_y]],
                                     wall_style, height=gate["clear_height_m"]))
        shapes.append(box("partition_culvert_cover", [x, y, gate["clear_height_m"] + gate["roof_thickness_m"] / 2],
                          [gate["length_m"] + 0.10, gate["clear_width_m"] + 2 * gate["wall_thickness_m"] + 0.10,
                           gate["roof_thickness_m"]], [0.62, 0.62, 0.59]))

    pipe = config["pipe"]
    shapes.extend(annular_segments("concrete_pipe", [*pipe["center_m"], pipe["outer_radius_m"]],
                                  pipe["inner_radius_m"], pipe["outer_radius_m"], pipe["length_m"],
                                  pipe["segments"], "x", [0.57, 0.58, 0.56], True))
    frame = config["frame"]
    x, y = frame["center_m"]
    half_x, half_y = frame["length_m"] / 2, frame["width_m"] / 2
    # Side walls run north-south, leaving both portals open below the roof.
    for side in (-1, 1):
        wall_x = x + side * (half_x - frame["wall_thickness_m"] / 2)
        wall_style = dict(style, thickness_m=frame["wall_thickness_m"])
        shapes.extend(brick_wall("culvert_side_" + str(side),
                                 [[wall_x, y - half_y], [wall_x, y + half_y]],
                                 wall_style, height=frame["height_m"]))
    roof = frame["roof_thickness_m"]
    shapes.append(box("culvert_cover_slab", [x, y, frame["height_m"] + roof / 2],
                      [frame["length_m"] + 0.12, frame["width_m"] + 0.12, roof], [0.63, 0.63, 0.60]))

    for helipad in config["helipads"]:
        x, y = helipad["center_m"]
        shapes.extend(annular_segments(helipad["name"] + "_ring", [x, y, 0.004],
                                      helipad["radius_m"] - 0.035, helipad["radius_m"],
                                      0.004, 72, "z", [0.93, 0.93, 0.90], False))
        for index, (dx, size) in enumerate([(-0.17, [0.075, 0.53, 0.006]),
                                           (0.17, [0.075, 0.53, 0.006]),
                                           (0, [0.34, 0.075, 0.006])]):
            shapes.append(box(helipad["name"] + "_H_" + str(index), [x + dx, y, 0.006],
                              size, [0.93, 0.93, 0.90], collision=False))
    sphere = config["sphere_landmark"]
    x, y = sphere["center_m"]
    shapes.append({"name": "sphere_base", "type": "cylinder", "position_m": [x, y, sphere["base_height_m"] / 2],
                   "size_m": [sphere["base_radius_m"], sphere["base_height_m"]], "rotation": None,
                   "color": [0.47, 0.48, 0.46], "collision": True})
    ball = trimesh.creation.icosphere(subdivisions=2, radius=sphere["radius_m"])
    shapes.append(mesh_shape("sphere_landmark", [x, y, sphere["base_height_m"] + sphere["radius_m"]],
                             ball.vertices, ball.faces, [0.57, 0.58, 0.56]))
    marker = config["blue_marker"]
    vertices = []
    for depth in (-0.005, 0.005):
        for index in range(8):
            angle = index * 2 * math.pi / 8
            vertices.append([depth, math.cos(angle) * marker["width_m"] / 2,
                             math.sin(angle) * marker["height_m"] / 2])
    faces = [[0, index + 1, index] for index in range(1, 7)]
    faces.extend([[8, 8 + index, 9 + index] for index in range(1, 7)])
    for index in range(8):
        nxt = (index + 1) % 8
        faces.extend([[index, nxt, nxt + 8], [index, nxt + 8, index + 8]])
    blue = mesh_shape("blue_octagon_marker", marker["position_m"], vertices, faces,
                      [0.40, 0.68, 0.73], collision=False)
    blue["alpha"] = marker["alpha"]
    shapes.append(blue)

    cube = trimesh.creation.box()
    angular_rock = trimesh.creation.icosphere(subdivisions=0, radius=0.5)
    for pile_index, pile in enumerate(config["rubble_piles"]):
        random = np.random.default_rng(711 + pile_index)
        total = pile["stone_count"] + 34
        for index in range(total):
            large = index < pile["stone_count"]
            radius = math.sqrt(random.uniform(0, 1))
            angle = index * 2.399963229728653
            extent = np.array(pile["footprint_m"]) * 0.5
            center = np.asarray(pile["center_m"]) + extent * radius * [math.cos(angle), math.sin(angle)]
            size = random.uniform(0.18, 0.43, 3) if large else random.uniform(0.04, 0.12, 3)
            size[2] *= 0.7
            template = cube if index % 3 == 0 else angular_rock
            vertices = template.vertices * size * random.uniform(0.73, 1.15, template.vertices.shape)
            yaw = random.uniform(0, 2 * math.pi)
            height = -float(vertices[:, 2].min()) + (0.16 * (1 - radius) if large else 0)
            shade = float(random.uniform(0.49, 0.65))
            shapes.append(mesh_shape(f"{pile['name']}_stone_{index}", [*center, height], vertices,
                                     template.faces, [shade, shade, shade * 0.97],
                                     rotation=[0, 0, 1, yaw]))
    return shapes

