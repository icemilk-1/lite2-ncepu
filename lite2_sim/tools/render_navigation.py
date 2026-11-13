"""Render a recorded navigation trajectory as an annotated MP4 replay."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

import imageio_ffmpeg
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
sys.path.insert(0, str(PROJECT / "controllers/sensor_capture"))

from mujoco_capture import render_options
from mujoco_navigation import MujocoVelocityExecutor


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--speed", type=float, default=8.0)
    parser.add_argument("--fps", type=int, default=20)
    args = parser.parse_args()
    if not math.isfinite(args.speed) or args.speed <= 0 or not 1 <= args.fps <= 60:
        parser.error("Speed must be positive and fps must be between 1 and 60.")
    output = args.run_directory.resolve()
    config = json.loads((output / "config_snapshot.json").read_text(encoding="utf-8"))
    summary = json.loads((output / "navigation_verification.json").read_text(encoding="utf-8"))
    trajectory_file = output / "slam_trajectory.csv"
    rows = np.genfromtxt(trajectory_file, delimiter=",", names=True)
    rows = np.atleast_1d(rows)
    if not len(rows) or not np.isfinite(rows["sim_time_s"]).all():
        raise ValueError("A finite recorded trajectory is required.")
    times = rows["sim_time_s"]
    duration = float(times[-1])
    model = mujoco.MjModel.from_xml_path(str(output / "scene.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    executor = MujocoVelocityExecutor(model, data, config)
    width, height = 1280, 960
    video = output / "navigation_replay.mp4"
    writer = imageio_ffmpeg.write_frames(str(video), (width, height), fps=args.fps,
        codec="libx264", quality=7, macro_block_size=16, output_params=["-movflags", "+faststart"])
    writer.send(None)
    font_path = Path("C:/Windows/Fonts/msyh.ttc")
    font = ImageFont.truetype(str(font_path), 24) if font_path.exists() else ImageFont.load_default()
    small_font = ImageFont.truetype(str(font_path), 19) if font_path.exists() else font
    camera = mujoco.MjvCamera()
    camera.lookat[:] = config.get("preview", {}).get("lookat_m", [0, 0, 0])
    camera.distance = config.get("preview", {}).get("distance_m", 16)
    camera.azimuth = config.get("preview", {}).get("azimuth_deg", 110)
    camera.elevation = config.get("preview", {}).get("elevation_deg", -65)
    options = render_options()
    frame_times = np.append(np.arange(0, duration, args.speed/args.fps), duration)
    path = np.column_stack((rows["truth_x_m_eval"], rows["truth_y_m_eval"], rows["ground_height_m_eval"] + .025))
    path_indices = [0]
    for index in range(1, len(path)):
        if np.linalg.norm(path[index, :2] - path[path_indices[-1], :2]) >= .10:
            path_indices.append(index)
    yaw = np.unwrap(rows["truth_yaw_rad_eval"])
    with (output / "velocity_commands.csv").open(encoding="utf-8") as stream:
        commands = list(csv.DictReader(stream))
    command_times = np.array([float(row["sim_time_s"]) for row in commands])
    statuses = {"mapping": "初始建图", "exploring": "探索通路", "navigating": "前往终点",
                "frontier_reached": "更新探索点", "goal_reached": "到点停车"}
    try:
        with mujoco.Renderer(model, height=height, width=width) as overview, mujoco.Renderer(model, height=240, width=424) as front:
            for frame_index, timestamp in enumerate(frame_times):
                x = float(np.interp(timestamp, times, rows["truth_x_m_eval"]))
                y = float(np.interp(timestamp, times, rows["truth_y_m_eval"]))
                heading = float(np.interp(timestamp, times, yaw))
                ground = float(np.interp(timestamp, times, rows["ground_height_m_eval"]))
                roll = float(np.interp(timestamp, times, rows["imu_roll_rad"]))
                pitch = float(np.interp(timestamp, times, rows["imu_pitch_rad"]))
                executor.set_pose(data, [x, y, heading], ground, roll, pitch)
                overview.update_scene(data, camera=camera, scene_option=options)
                previous = None
                for index in path_indices:
                    if times[index] > timestamp:
                        break
                    if previous is not None and overview.scene.ngeom < overview.scene.maxgeom:
                        geom = overview.scene.geoms[overview.scene.ngeom]
                        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3),
                                           np.eye(3).reshape(-1), np.array([.10, .55, 1., 1.], dtype=np.float32))
                        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, .014, path[previous], path[index])
                        overview.scene.ngeom += 1
                    previous = index
                canvas = Image.fromarray(overview.render().copy())
                front.update_scene(data, camera="front_rgb", scene_option=options)
                inset = Image.fromarray(front.render().copy()).resize((382, 216))
                canvas.paste(inset, (width-406, height-252))
                draw = ImageDraw.Draw(canvas)
                draw.rounded_rectangle((20, 18, 710, 111), radius=12, fill=(22, 27, 33))
                draw.text((36, 28), "Lite2 在线建图导航 · 记录轨迹回放", font=font, fill="white")
                command_index = max(0, int(np.searchsorted(command_times, timestamp, side="right")-1))
                status = statuses.get(commands[command_index]["status"], commands[command_index]["status"])
                draw.text((36, 68), "%s · 仿真 %.1f / %.1f 秒 · %.0f 倍速" % (status, timestamp, duration, args.speed),
                          font=small_font, fill=(195, 219, 244))
                draw.text((width-402, height-282), "机载相机视角", font=small_font, fill="white")
                draw.text((24, height-36), "速度级运动模型 · 四肢保持配置姿态", font=small_font, fill="white")
                writer.send(np.asarray(canvas, dtype=np.uint8).tobytes())
                if frame_index == len(frame_times)-1:
                    canvas.save(output / "navigation_replay_poster.png")
                    for _ in range(args.fps*2):
                        writer.send(np.asarray(canvas, dtype=np.uint8).tobytes())
                if frame_index % 80 == 0:
                    print("VIDEO_FRAME %d/%d" % (frame_index, len(frame_times)), flush=True)
    finally:
        writer.close()
    manifest = {"video": video.name, "source_run": str(output), "source_result": summary["result"],
                "source_trajectory_sha256": hashlib.sha256(trajectory_file.read_bytes()).hexdigest(),
                "replay_type": "recorded_body_trajectory_with_configured_joint_pose",
                "speed_multiplier": args.speed, "fps": args.fps, "resolution": [width, height],
                "simulation_duration_s": duration, "video_duration_s": len(frame_times)/args.fps+2,
                "mujoco_version": mujoco.__version__, "imageio_ffmpeg_version": imageio_ffmpeg.__version__}
    (output / "video_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print("VIDEO_READY " + str(video), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
