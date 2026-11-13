"""Convert the supplied Lite2 URDF and shared scene geometry to MJCF."""

import hashlib
import io
import json
import math
import os
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import trimesh

from .prepare_model import extract_model
from .scene_geometry import build_geometry


PROJECT = Path(__file__).resolve().parents[1]
SIMULATION = {"physics_step_s": 0.002, "control_period_s": 0.02,
              "kp": 40.0, "kv": 2.0, "settle_time_s": 5.0,
              "parameter_status": "provisional_simulation_pose_servo"}


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def atomic_write(path, content):
    path = Path(path)
    temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    try:
        temporary.write_bytes(content)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def numbers(values):
    return " ".join(format(float(value), ".12g") for value in values)


def convert_mesh(source):
    """Drop unsupported DAE materials from an in-memory copy of the geometry."""
    root = ET.parse(source).getroot()
    namespace = "{http://www.collada.org/2005/11/COLLADASchema}"
    material_tags = {namespace + name for name in
                     ("library_effects", "library_materials", "bind_material")}
    for parent in root.iter():
        for child in list(parent):
            if child.tag in material_tags:
                parent.remove(child)
    scene = trimesh.load_scene(io.BytesIO(ET.tostring(root)), file_type="dae")
    mesh = scene.to_geometry()
    if not len(mesh.vertices) or not np.isfinite(mesh.vertices).all():
        raise ValueError("Invalid source mesh: " + str(source))
    unit = float(root.find(namespace + "asset/" + namespace + "unit").get("meter", "1"))
    if not math.isclose(unit, 1.0):
        raise ValueError("The supplied Lite2 meshes must use meters.")
    payload = mesh.export(file_type="stl")
    converted = trimesh.load(io.BytesIO(payload), file_type="stl", process=False)
    error = float(np.max(np.abs(converted.bounds - mesh.bounds)))
    if error > 1e-5:
        raise ValueError("Mesh conversion changed the source dimensions.")
    return payload, {"bounds_m": mesh.bounds.tolist(), "conversion_bounds_error_m": error}


def prepare_robot():
    source = extract_model()
    root = ET.parse(source).getroot()
    source_files = {str(source.relative_to(PROJECT)): sha256(source.read_bytes())}
    for element in root.findall(".//mesh"):
        path = (source.parent / element.get("filename")).resolve()
        source_files[str(path.relative_to(PROJECT))] = sha256(path.read_bytes())
    identity = {"source_files": source_files, "builder": sha256(Path(__file__).read_bytes()),
                "mujoco": mujoco.__version__, "trimesh": trimesh.__version__}
    digest = sha256(json.dumps(identity, sort_keys=True).encode())
    cache = PROJECT / "mujoco" / "models" / digest[:20]
    manifest_path = cache / "model_manifest.json"
    if manifest_path.exists():
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, UnicodeError):
            manifest = {}
        if manifest.get("source_digest") == digest and manifest.get("generated_files") and all(
                (cache / name).is_file() and sha256((cache / name).read_bytes()) == value
                for name, value in manifest["generated_files"].items()):
            return cache, manifest

    cache.mkdir(parents=True, exist_ok=True)
    payloads, meshes = {}, {}
    for element in root.findall(".//mesh"):
        original = (source.parent / element.get("filename")).resolve()
        name = original.with_suffix(".stl").name if original.suffix == ".dae" else original.name
        if name not in payloads:
            if original.suffix == ".dae":
                payloads[name], meshes[name] = convert_mesh(original)
            else:
                payloads[name] = original.read_bytes()
        element.set("filename", name)

    inertia = root.find("link[@name='INERTIA']")
    fixed = root.find("joint[@name='Torso2Inertia']")
    if fixed.get("type") != "fixed" or fixed.find("origin") is not None:
        raise ValueError("Torso inertia must be attached by the supplied zero-offset fixed joint.")
    root.find("link[@name='TORSO']").append(inertia.find("inertial"))
    root.remove(inertia)
    root.remove(fixed)
    bodies = {}
    for link in root.findall("link"):
        inertial = link.find("inertial")
        origin = inertial.find("origin")
        inertia_values = inertial.find("inertia").attrib
        tensor = [[float(inertia_values[key]) for key in row]
                  for row in (("ixx", "ixy", "ixz"), ("ixy", "iyy", "iyz"), ("ixz", "iyz", "izz"))]
        bodies[link.get("name")] = {
            "mass_kg": float(inertial.find("mass").get("value")),
            "inertial_origin_xyz_m": [float(value) for value in origin.get("xyz", "0 0 0").split()],
            "inertial_origin_rpy_rad": [float(value) for value in origin.get("rpy", "0 0 0").split()],
            "inertia_tensor_kg_m2": tensor,
        }
    joints = {}
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        joints[joint.get("name")] = {
            "axis": [float(value) for value in joint.find("axis").get("xyz").split()],
            "range_rad": [float(limit.get("lower")), float(limit.get("upper"))],
            "effort_nm": float(limit.get("effort")),
        }
    ET.SubElement(ET.SubElement(root, "mujoco"), "compiler",
                  discardvisual="false", fusestatic="false", strippath="false")
    spec = mujoco.MjSpec.from_string(ET.tostring(root, encoding="unicode"), assets=payloads)
    compiled = spec.compile()
    payloads["lite2.xml"] = spec.to_xml().encode()
    manifest = {**identity, "source_digest": digest, "robot": "DEEP Robotics Lite2",
                "source_model_mass_kg": float(compiled.body_mass.sum()), "joints": joints, "bodies": bodies,
                "meshes": meshes, "appearance": "uniform_preview_material",
                "collision_model": "source_primitives_and_mesh_convex_hulls",
                "generated_files": {name: sha256(payload) for name, payload in payloads.items()}}
    for name, payload in payloads.items():
        atomic_write(cache / name, payload)
    atomic_write(manifest_path, json.dumps(manifest, indent=2).encode())
    return cache, manifest


def build_scene(config, scene_id, output):
    cache, manifest = prepare_robot()
    root = ET.parse(cache / "lite2.xml").getroot()
    root.find("compiler").set("meshdir", cache.as_posix())
    ET.SubElement(root, "option", timestep="0.002", integrator="implicitfast",
                  gravity="0 0 -9.81", cone="elliptic", solver="Newton", iterations="50")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="1600", offheight="1200")
    ET.SubElement(visual, "map", znear="0.001", zfar="5")
    ET.SubElement(visual, "headlight", ambient="0.35 0.35 0.35", diffuse="0.45 0.45 0.45", specular="0 0 0")
    ET.SubElement(root, "statistic", center="0 0 0", extent="10")
    world = root.find("worldbody")
    torso = world.find("body[@name='TORSO']")
    torso.set("pos", numbers(config["robot_position_m"]))
    ET.SubElement(torso, "freejoint", name="root")
    fixture = ET.SubElement(world, "body", name="base_fixture", mocap="true",
                            pos=numbers(config["robot_position_m"]))
    if config.get("navigation_execution"):
        ET.SubElement(fixture, "geom", name="navigation_envelope", type="box",
                      size="0.30 0.22 0.23", group="5", contype="0", conaffinity="0",
                      rgba="0 0 0 0")
    ET.SubElement(ET.SubElement(root, "equality"), "weld", name="base_fixture_weld",
                  body1="TORSO", body2="base_fixture", solref="0.004 1")
    for geom in torso.iter("geom"):
        if geom.get("contype") == "0" and geom.get("conaffinity") == "0":
            geom.set("group", "2")
            geom.set("rgba", "0.22 0.25 0.29 1")
        else:
            geom.set("group", "0")

    geometry = build_geometry(config, scene_id)
    assets = root.find("asset")
    ET.SubElement(assets, "material", name="arena_matte", rgba="1 1 1 1",
                  specular="0", shininess="0", reflectance="0")
    for shape in geometry:
        attributes = {"name": shape["name"], "type": shape["type"],
                      "pos": numbers(shape["position_m"]),
                      "rgba": numbers([*shape["color"], shape.get("alpha", 1)]),
                      "material": "arena_matte",
                      "group": "3" if shape["collision"] else "4"}
        if shape["type"] == "mesh":
            mesh_name = "scene_mesh_" + shape["name"]
            ET.SubElement(assets, "mesh", name=mesh_name,
                          vertex=numbers(np.asarray(shape["vertices_m"]).reshape(-1)),
                          face=" ".join(str(index) for face in shape["faces"] for index in face))
            attributes["mesh"] = mesh_name
        else:
            size = np.asarray(shape["size_m"], dtype=float)
            size = size / 2 if shape["type"] == "box" else [size[0], size[1] / 2]
            attributes["size"] = numbers(size)
        if shape["rotation"] is not None:
            attributes["axisangle"] = numbers(shape["rotation"])
        if not shape["collision"]:
            attributes.update(contype="0", conaffinity="0")
        ET.SubElement(world, "geom", **attributes)
    ET.SubElement(world, "light", pos="-3 -3 8", dir="0 0 -1", directional="true",
                  diffuse="0.45 0.45 0.45", specular="0 0 0", ambient="0.1 0.1 0.1")
    ET.SubElement(world, "light", pos="4 3 6", dir="-0.3 -0.3 -1", diffuse="0.2 0.2 0.2", specular="0 0 0")
    ET.SubElement(world, "camera", name="plan_view", pos="0 0 18", xyaxes="1 0 0 0 1 0",
                  projection="orthographic", fovy="11.4")
    camera = config["camera"]
    fovy = math.degrees(2 * math.atan(camera["height"] / (2 * camera["focal_length_px"])))
    ET.SubElement(torso, "camera", name="front_rgb", pos=numbers(camera["mount_position_m"]),
                  xyaxes="0 -1 0 0 0 1", fovy=str(fovy))
    ET.SubElement(torso, "site", name="imu_site", size="0.005", rgba="0 0 0 0")
    if config.get("lidar"):
        ET.SubElement(torso, "site", name="front_lidar", size="0.005",
                      pos=numbers(config["lidar"]["mount_xyz_m"]), rgba="0 0 0 0")
    sensors = ET.SubElement(root, "sensor")
    ET.SubElement(sensors, "gyro", name="body_gyro", site="imu_site")
    ET.SubElement(sensors, "accelerometer", name="body_accelerometer", site="imu_site")
    ET.SubElement(sensors, "framequat", name="body_orientation", objtype="site", objname="imu_site")
    actuators = ET.SubElement(root, "actuator")
    if set(config["joint_positions_rad"]) != set(manifest["joints"]):
        raise ValueError("Configured joint names differ from the manufacturer URDF.")
    for name, joint in manifest["joints"].items():
        target = config["joint_positions_rad"][name]
        if not joint["range_rad"][0] <= target <= joint["range_rad"][1]:
            raise ValueError("Initial joint position exceeds its URDF limit: " + name)
        ET.SubElement(actuators, "position", name=name + "_servo", joint=name,
                      kp=str(SIMULATION["kp"]), kv=str(SIMULATION["kv"]),
                      ctrlrange=numbers(joint["range_rad"]),
                      forcerange=numbers([-joint["effort_nm"], joint["effort_nm"]]))
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    for name, target in config["joint_positions_rad"].items():
        data.qpos[model.joint(name).qposadr[0]] = target
        data.ctrl[model.actuator(name + "_servo").id] = target
    ET.SubElement(ET.SubElement(root, "keyframe"), "key", name="initial_pose",
                  qpos=numbers(data.qpos), ctrl=numbers(data.ctrl))
    ET.indent(root)
    path = Path(output) / "scene.xml"
    path.write_text(ET.tostring(root, encoding="unicode"), encoding="utf-8")
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    provenance = {**manifest, "backend": "mujoco", "scene_id": scene_id,
                  "scene_sha256": sha256(path.read_bytes()), "geometry": geometry,
                  "geometry_status": config["geometry_status"], "simulation": SIMULATION}
    (Path(output) / "model_manifest.json").write_text(json.dumps(provenance, indent=2), encoding="utf-8")
    return model, data, provenance

