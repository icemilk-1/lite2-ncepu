"""Prepare the supplied Lite2 model and a reproducible Webots sensor scene."""

import argparse
import json
import math
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath


try:
    from .scene_geometry import build_geometry, sensor_geometry
except ImportError:
    from scene_geometry import build_geometry, sensor_geometry


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


def render_geometry(shape):
    if shape["type"] == "mesh":
        vertices = ", ".join(numbers(vertex) for vertex in shape["vertices_m"])
        faces = ", ".join(" ".join(str(index) for index in face) + " -1" for face in shape["faces"])
        geometry = f"IndexedFaceSet {{ coord Coordinate {{ point [ {vertices} ] }} coordIndex [ {faces} ] }}"
        rotation = "" if shape["rotation"] is None else "rotation " + numbers(shape["rotation"])
        collision = "boundingObject " + geometry if shape["collision"] else ""
        return f'''Solid {{
  translation {numbers(shape['position_m'])}
  {rotation}
  name "{shape['name']}"
  children [ Shape {{ appearance PBRAppearance {{ baseColor {numbers(shape['color'])} roughness 0.9 metalness 0 transparency {1 - shape.get('alpha', 1)} }} geometry {geometry} }} ]
  {collision}
}}
'''
    if shape["type"] == "box":
        text = box(shape["name"], shape["position_m"], shape["size_m"],
                   shape["color"], shape["rotation"])
        if not shape["collision"]:
            text = re.sub(r"(?m)^  boundingObject Box .*\n", "", text)
            text = text.replace("roughness 0.85", "roughness 0.9")
            text = text.replace(" metalness 0", "")
        return text
    radius, height = shape["size_m"]
    geometry = f"Cylinder {{ radius {radius} height {height} }}"
    return f'''Solid {{
  translation {numbers(shape['position_m'])}
  rotation 1 0 0 1.57079632679
  name "{shape['name']}"
  children [ Shape {{ appearance PBRAppearance {{ baseColor {numbers(shape['color'])} roughness 0.7 }} geometry {geometry} }} ]
  boundingObject {geometry}
}}
'''


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

    archive_roots = [PROJECT.parent / "resources" / "绝影机器人资料",
                     PROJECT.parent / "绝影机器人资料"]
    archives = [archive for root in archive_roots
                for archive in root.rglob("LiteV2版模型文件.rar")]
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
    shapes = [render_geometry(shape) for shape in sensor_geometry(config)]

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
    solids = [render_geometry(shape) for shape in build_geometry(config, "ruins_reference")]

    viewpoint = config.get("viewpoint", {
        "field_of_view_rad": 1.05, "orientation": [-0.301, 0.617, 0.733, 1.174],
        "position_m": [-10.5, -13.4, 17.0]})
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
