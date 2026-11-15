"""Render a recorded navigation run to an MP4 by replaying the truth trajectory.

The velocity-level fixture already recorded every pose, so no re-simulation is
needed: load the run's ``scene.xml``, push the evaluated truth pose into the free
root joint frame by frame and pipe the RGB frames to the ffmpeg binary that ships
with ``imageio_ffmpeg``.

Usage:
    python tools/render_run_video.py runs/<run> [--out video.mp4] [--fps 12]
                                          [--step 0.2] [--camera plan_view]
                                          [--follow] [--width 960] [--height 720]
"""
import argparse
import csv
import json
import math
import subprocess
import sys
from pathlib import Path

import imageio_ffmpeg
import mujoco
import numpy as np

PROJECT = Path(__file__).resolve().parents[1]


def yaw_quaternion(yaw):
    return np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])


def load_trajectory(run):
    rows = list(csv.DictReader((run / "slam_trajectory.csv").open(encoding="utf-8")))
    poses = [(float(row["sim_time_s"]), float(row["truth_x_m_eval"]),
              float(row["truth_y_m_eval"]), float(row["truth_yaw_rad_eval"]),
              float(row["ground_height_m_eval"])) for row in rows]
    return rows, poses


def stage_timeline(run):
    path = run / "course_events.jsonl"
    if not path.exists():
        return []
    events = [json.loads(line) for line in path.open(encoding="utf-8") if line.strip()]
    return [e for e in events if e["event"] in ("checkpoint_started", "stage_completed",
                                                "passage_rejected", "branch_closed",
                                                "stage_failed", "recovery_exhausted")]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run")
    parser.add_argument("--out", default=None)
    parser.add_argument("--fps", type=float, default=12.0)
    parser.add_argument("--step", type=float, default=0.2)
    parser.add_argument("--camera", default=None,
                        help="scene camera name; omit for a free tracking camera")
    parser.add_argument("--follow", action="store_true", default=True)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--base-height", type=float, default=0.33)
    parser.add_argument("--distance", type=float, default=9.0)
    parser.add_argument("--elevation", type=float, default=-60.0)
    parser.add_argument("--azimuth", type=float, default=120.0)
    parser.add_argument("--no-overlay", action="store_true")
    args = parser.parse_args()

    run = Path(args.run)
    if not run.is_absolute():
        run = PROJECT / run
    out = Path(args.out) if args.out else run / "navigation.mp4"
    if not out.is_absolute():
        out = PROJECT / out

    model = mujoco.MjModel.from_xml_path(str(run / "scene.xml"))
    data = mujoco.MjData(model)
    config_path = run / 'config_snapshot.json'
    if config_path.exists():
        config = json.loads(config_path.read_text(encoding='utf-8'))
        for name, target in config.get('joint_positions_rad', {}).items():
            data.qpos[model.joint(name).qposadr[0]] = target
    rows, poses = load_trajectory(run)
    moving_obstacle = None
    if rows and 'obstacle_x_m' in rows[0]:
        moving_obstacle = model.geom('rl_obstacle').id
    if not poses:
        raise SystemExit("no trajectory rows in %s" % run)

    camera = None
    if args.camera and args.camera in [model.camera(i).name for i in range(model.ncam)]:
        camera = args.camera
    free_camera = mujoco.MjvCamera()
    free_camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    free_camera.azimuth = args.azimuth
    free_camera.elevation = args.elevation
    free_camera.distance = args.distance
    free_camera.lookat[:] = [poses[0][1], poses[0][2], 0.2]
    options = mujoco.MjvOption()
    # Groups 0/1 are the collision-only stand-ins, 2-4 are the visible model and
    # the floor; group 5 is the giant enclosure that would hide everything.
    options.geomgroup[:] = [0, 0, 1, 1, 1, 0]
    options.sitegroup[:] = 0
    stage_by_time = []
    for event in stage_timeline(run):
        stage_by_time.append((event["timestamp_s"], event.get("stage_id") or "",
                              event["event"]))

    def stage_at(time_s):
        label = ""
        for timestamp, stage, event in stage_by_time:
            if time_s + 1e-9 >= timestamp and event in ("checkpoint_started", "stage_completed"):
                label = stage
        return label

    encoder = [
        imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "error",
        "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-s", "%dx%d" % (args.width, args.height), "-r", str(args.fps), "-i", "-",
        "-an", "-c:v", "libx264", "-preset", "medium", "-crf", "20",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(out),
    ]
    process = subprocess.Popen(encoder, stdin=subprocess.PIPE)
    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    written = 0
    next_time = poses[0][0]
    stage_events = stage_timeline(run)
    from PIL import Image, ImageDraw
    try:
        for row, (time_s, x, y, yaw, ground) in zip(rows, poses):
            if time_s + 1e-9 < next_time:
                continue
            next_time = time_s + args.step
            data.qpos[0:3] = [x, y, ground + args.base_height]
            data.qpos[3:7] = yaw_quaternion(yaw)
            if moving_obstacle is not None:
                model.geom_pos[moving_obstacle] = [float(row[key]) for key in ('obstacle_x_m', 'obstacle_y_m', 'obstacle_z_m')]
            mujoco.mj_forward(model, data)
            if args.follow:
                free_camera.lookat[:] = [x, y, ground + 0.2]
            renderer.update_scene(data, camera=(camera if camera is not None else free_camera),
                                  scene_option=options)
            rgb = renderer.render()
            if not args.no_overlay:
                image = Image.fromarray(rgb)
                draw = ImageDraw.Draw(image)
                caption = "t=%6.1fs  stage=%s" % (time_s, stage_at(time_s) or "-")
                draw.rectangle([(0, 0), (image.width, 22)], fill=(0, 0, 0))
                draw.text((6, 5), caption, fill=(255, 230, 120))
                rgb = np.asarray(image)
            process.stdin.write(np.ascontiguousarray(rgb, dtype=np.uint8).tobytes())
            written += 1
    finally:
        renderer.close()
        process.stdin.close()
        process.wait()
    duration = poses[-1][0] - poses[0][0]
    print(json.dumps({"output": str(out), "frames": written, "fps": args.fps,
                      "sim_duration_s": round(duration, 2),
                      "video_duration_s": round(written / args.fps, 2),
                      "stages": [{"t": round(e["timestamp_s"], 1), "event": e["event"],
                                  "stage": e.get("stage_id")} for e in stage_events]},
                     ensure_ascii=False, indent=1))


if __name__ == "__main__":
    sys.exit(main())
