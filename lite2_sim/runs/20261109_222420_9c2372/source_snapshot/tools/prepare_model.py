"""Prepare the supplied Lite2 model and a reproducible Webots sensor scene."""

import argparse
import json
import math
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath


PROJECT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PROJECT / "config" / "sensor_lab.json"


def numbers(values):
    return " ".join(str(value) for value in values)


def box(name, position, size, color, rotation=None):
    rotation_field = "" if rotation is None else "  rotation " + numbers(rotation) + "\n"
    return f'''Solid {{
  translation {numbers(position)}
{rotation_field}  name "{name}"
  children [
    Shape {{
      appearance PBRAppearance {{ baseColor {numbers(color)} roughness 0.85 metalness 0 }}
      geometry Box {{ size {numbers(size)} }}
    }}
  ]
  boundingObject Box {{ size {numbers(size)} }}
}}
'''


def segment_wall(name, start, end, width, height, color):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    if length < 1e-5:
        return ""
    angle = math.atan2(dy, dx)
    center = [(start[0] + end[0]) / 2, (start[1] + end[1]) / 2, height / 2]
    return box(name, center, [length, width, height], color, [0, 0, 1, angle])


def build_robot_node(config, node_name):
    camera = config["camera"]
    fov = 2 * math.atan(camera["width"] / (2 * camera["focal_length_px"]))
    config_path = (PROJECT / "config" / (config.get("scene_id", "sensor_lab") + ".json")).as_posix()
    lidar = config.get("lidar")
    lidar_node = ""
    if lidar:
        lidar_node = f'''\n          Lidar {{
            name "front_lidar"
            translation {numbers(lidar['mount_xyz_m'])}
            horizontalResolution {lidar['horizontal_resolution']}
            fieldOfView {lidar['field_of_view_rad']}
            verticalFieldOfView {lidar['vertical_field_of_view_rad']}
            numberOfLayers {lidar.get('number_of_layers', 1)}
            near 0.01
            minRange {lidar['min_range_m']}
            maxRange {lidar['max_range_m']}
            type "{lidar['type']}"
            projection "{lidar.get('projection', 'cylindrical')}"
            noise {lidar['noise_m']}
            defaultFrequency {lidar['frequency_hz']}
          }}'''
    return f'''DEF ROBOT_FIXTURE Solid {{
  translation {numbers(config['robot_position_m'])}
  name "{node_name}_fixture"
  children [
    HingeJoint {{
      jointParameters HingeJointParameters {{
        axis 0 0 1
        minStop -0.000001
        maxStop 0.000001
      }}
      endPoint DEF LITE2 Lite2Body {{
        name "{node_name}"
        controller "sensor_capture"
        supervisor TRUE
        customData "{config_path}"
        toolSlot [
          Pose {{
            translation {numbers(camera['mount_position_m'])}
            children [
              Camera {{
                name "front_rgb"
                width {camera['width']}
                height {camera['height']}
                fieldOfView {fov}
                near 0.02
                far {camera['max_depth_m']}
              }}
              RangeFinder {{
                name "front_depth"
                width {camera['width']}
                height {camera['height']}
                fieldOfView {fov}
                near 0.02
                minRange {camera['min_depth_m']}
                maxRange {camera['max_depth_m']}
              }}
            ]
          }}
{lidar_node}
          InertialUnit {{ name "body_orientation" }}
          Accelerometer {{ name "body_accelerometer" }}
          Gyro {{ name "body_gyro" }}
        ]
      }}
    }}
  ]
}}
'''


def extract_model():
    destination = PROJECT / "assets"
    urdf = destination / "JueyingMiniLiteV2" / "urdf" / "MiniLiteV2_Rsm.urdf"
    if urdf.exists():
        return urdf

    archives = list((PROJECT.parent / "绝影机器人资料").rglob("LiteV2版模型文件.rar"))
    if len(archives) != 1:
        raise RuntimeError("Expected one supplied LiteV2 model archive.")
    listing = subprocess.run(
        ["tar", "-tf", str(archives[0])],
        check=True, capture_output=True, text=True, encoding="utf-8",
    ).stdout.splitlines()
    for entry in listing:
        path = PurePosixPath(entry)
        if path.is_absolute() or ".." in path.parts or ":" in entry or "\\" in entry:
            raise RuntimeError("Unexpected model archive path: " + entry)
        if not path.parts or path.parts[0] != "JueyingMiniLiteV2":
            raise RuntimeError("Unexpected model archive root: " + entry)
    destination.mkdir(parents=True, exist_ok=True)
    subprocess.run(["tar", "-xf", str(archives[0]), "-C", str(destination)], check=True)
    if not urdf.exists():
        raise RuntimeError("The supplied URDF was not extracted.")
    return urdf


def convert_model(urdf, config):
    output = PROJECT / "protos" / "Lite2Body.proto"
    output.parent.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(urdf)
    root = tree.getroot()
    torso = root.find("link[@name='TORSO']")
    inertia_link = root.find("link[@name='INERTIA']")
    inertia_joint = root.find("joint[@name='Torso2Inertia']")
    if inertia_joint.find("origin") is not None or inertia_joint.attrib["type"] != "fixed":
        raise RuntimeError("The supplied inertia joint is expected to be fixed at the root origin.")
    # This fixed child carries only the torso inertia. Merge it into the root.
    torso.append(inertia_link.find("inertial"))
    root.remove(inertia_link)
    root.remove(inertia_joint)
    prepared = urdf.with_name("MiniLiteV2_webots.urdf")
    tree.write(prepared, encoding="utf-8", xml_declaration=True)
    joints = root.findall("joint")
    positions = config["joint_positions_rad"]
    movable = [joint for joint in joints if joint.attrib["type"] != "fixed"]
    if {joint.attrib["name"] for joint in movable} != set(positions):
        raise RuntimeError("Configured joints do not match the supplied URDF.")
    for joint in movable:
        limit = joint.find("limit")
        position = positions[joint.attrib["name"]]
        if not float(limit.attrib["lower"]) <= position <= float(limit.attrib["upper"]):
            raise RuntimeError("Initial joint position exceeds the URDF limit.")

    initial = [positions[joint.attrib["name"]] for joint in movable]
    subprocess.run([
        sys.executable, "-m", "urdf2webots.importer",
        "--input=" + str(prepared), "--output=" + str(output),
        "--tool-slot=TORSO", "--init-pos=" + json.dumps(initial),
        "--link-to-def", "--joint-to-def", "--target=R2025a",
    ], check=True)

    content = output.read_text(encoding="utf-8")
    # Expose the original root physics so the sensor lab can pin the base.
    pattern = r"(?m)^    physics Physics \{\n(?:      [^\n]*\n)*    \}"
    matches = list(re.finditer(pattern, content))
    if len(matches) != 1:
        raise RuntimeError("Expected exactly one root Physics block in the converted model.")
    physics = matches[0].group().strip().removeprefix("physics ")
    content = re.sub(pattern, "    physics IS basePhysics", content)
    content = content.replace(
        "PROTO Lite2Body [\n", "PROTO Lite2Body [\n  field SFNode basePhysics " + physics + "\n", 1,
    )
    # Keep the manufacturer's asset provenance separate from the converter's license.
    content = re.sub(r"(?m)^# license(?: url)?:.*\n", "", content)
    content = content.replace(
        "# This is a proto file", "# Model source: user-supplied DEEP Robotics Lite2 archive.\n# This is a proto file", 1,
    )
    content = re.sub(
        r'(?m)^(\s*url )"([^"\n]+)"',
        lambda match: match.group(1) + '"' + match.group(2).replace("\\", "/") + '"',
        content,
    )
    output.write_text(content, encoding="utf-8")


def build_world(config):
    camera = config["camera"]
    fov = 2 * math.atan(camera["width"] / (2 * camera["focal_length_px"]))
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

    # Webots R2025 cameras look along +X with +Z up, matching the body frame.
    world = f'''#VRML_SIM R2025a utf8
EXTERNPROTO "../protos/Lite2Body.proto"
WorldInfo {{
  title "Lite2 Sensor Lab"
  basicTimeStep 16
  coordinateSystem "ENU"
}}
Viewpoint {{
  orientation -0.219230394 0.529268991 0.819641610 0.935851804
  position -6.3 -5.5 2.5
}}
Background {{ skyColor [ 0.76 0.83 0.9 ] }}
DirectionalLight {{ direction 0.2 0.3 -1 intensity 1.5 ambientIntensity 0.5 }}
{''.join(shapes)}
Solid {{
  translation {numbers(config['robot_position_m'])}
  name "sensor_lab_fixture"
  children [
    HingeJoint {{
      jointParameters HingeJointParameters {{
        axis 0 0 1
        minStop -0.000001
        maxStop 0.000001
      }}
      endPoint DEF LITE2 Lite2Body {{
  name "lite2_sensor_lab"
  controller "sensor_capture"
  supervisor TRUE
  customData "{(PROJECT / 'config' / 'sensor_lab.json').as_posix()}"
  toolSlot [
    Pose {{
      translation {numbers(camera['mount_position_m'])}
      children [
        Camera {{
          name "front_rgb"
          width {camera['width']}
          height {camera['height']}
          fieldOfView {fov}
          near 0.02
          far {camera['max_depth_m']}
        }}
        RangeFinder {{
          name "front_depth"
          width {camera['width']}
          height {camera['height']}
          fieldOfView {fov}
          near 0.02
          minRange {camera['min_depth_m']}
          maxRange {camera['max_depth_m']}
        }}
      ]
    }}
    InertialUnit {{ name "body_orientation" }}
    Accelerometer {{ name "body_accelerometer" }}
    Gyro {{ name "body_gyro" }}
  ]
      }}
    }}
  ]
}}
'''
    path = PROJECT / "worlds" / "sensor_lab.wbt"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(world, encoding="utf-8")
    runtime = PROJECT / "controllers" / "sensor_capture" / "runtime.ini"
    runtime.write_text("[python]\nCOMMAND = " + Path(sys.executable).as_posix() + "\n", encoding="utf-8")
    return path


def build_ruins_world(config):
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

    # Markers and targets are visual placeholders until official definitions arrive.
    for marker in config["markers"]:
        color = [0.12, 0.65, 0.2] if marker["kind"] == "start" else [0.2, 0.3, 0.85]
        mx, my = marker["position_m"]
        marker_boxes = [
            ([mx - 0.10, my, 0.003], [0.055, 0.36, 0.006]),
            ([mx + 0.10, my, 0.003], [0.055, 0.36, 0.006]),
            ([mx, my, 0.003], [0.24, 0.055, 0.006]),
        ]
        for part_index, (position, size) in enumerate(marker_boxes):
            solids.append(f'''Solid {{
  translation {numbers(position)}
  name "{marker['name']}_{part_index}"
  children [ Shape {{ appearance PBRAppearance {{ baseColor {numbers(color)} roughness 0.9 }} geometry Box {{ size {numbers(size)} }} }} ]
}}
''')
    for target in config["target_placeholders"]:
        tx, ty, tz = target["position_m"]
        solids.append(f'''Solid {{
  translation {numbers([tx, ty, tz])}
  rotation 1 0 0 1.57079632679
  name "{target['name']}"
  children [ Shape {{ appearance PBRAppearance {{ baseColor 0.9 0.58 0.1 roughness 0.7 }} geometry Cylinder {{ radius 0.12 height 0.38 }} }} ]
  boundingObject Cylinder {{ radius 0.12 height 0.38 }}
}}
''')

    viewpoint = config["viewpoint"]
    world = f'''#VRML_SIM R2025a utf8
EXTERNPROTO "../protos/Lite2Body.proto"
WorldInfo {{ title "Lite2 Ruins Reference Course" basicTimeStep 16 coordinateSystem "ENU" }}
Viewpoint {{ fieldOfView {viewpoint['field_of_view_rad']} orientation {numbers(viewpoint['orientation'])} position {numbers(viewpoint['position_m'])} }}
Background {{ skyColor [ 0.76 0.83 0.9 ] }}
DirectionalLight {{ direction 0.2 0.3 -1 intensity 1.5 ambientIntensity 0.5 }}
{''.join(solids)}
{build_robot_node(config, 'lite2_ruins_reference')}
'''
    path = PROJECT / "worlds" / "ruins_reference.wbt"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(world, encoding="utf-8")
    source_project = PROJECT / "worlds" / ".sensor_lab.wbproj"
    project_path = PROJECT / "worlds" / ".ruins_reference.wbproj"
    if source_project.exists():
        project = re.sub(
            r"(?m)^renderingDevicePerspectives:.*\n", "",
            source_project.read_text(encoding="utf-8"),
        )
        # Windows rejects truncating an existing hidden file with mode "w".
        with project_path.open("r+" if project_path.exists() else "w", encoding="utf-8") as stream:
            stream.write(project)
            stream.truncate()
    runtime = PROJECT / "controllers" / "sensor_capture" / "runtime.ini"
    runtime.write_text("[python]\nCOMMAND = " + Path(sys.executable).as_posix() + "\n", encoding="utf-8")
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", choices=["sensor_lab", "ruins_reference"], default="sensor_lab")
    scene = parser.parse_args().scene
    config_path = PROJECT / "config" / (scene + ".json")
    config = json.loads(config_path.read_text(encoding="utf-8"))
    urdf = extract_model()
    convert_model(urdf, config)
    world = build_ruins_world(config) if scene == "ruins_reference" else build_world(config)
    print("Prepared", world)


if __name__ == "__main__":
    main()
