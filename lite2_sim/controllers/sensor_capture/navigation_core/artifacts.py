"""Run-side map rendering and export; excluded from the portable core."""

import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


def save_map(grid, output, path, start, goal, radius_m, occupied_threshold, trajectory=None):
    output = Path(output)
    occupied = grid.log_odds >= occupied_threshold
    free = grid.log_odds <= -0.18
    image = np.full((grid.height, grid.width), 205, dtype=np.uint8)
    image[free] = 250
    image[occupied] = 25
    pgm = np.flipud(image)
    with (output / "slam_map.pgm").open("wb") as stream:
        stream.write(("P5\n%d %d\n255\n" % (grid.width, grid.height)).encode("ascii"))
        stream.write(pgm.tobytes())
    (output / "slam_map.yaml").write_text(
        "image: slam_map.pgm\n"
        "resolution: %s\n" % grid.resolution
        + "origin: [%s, %s, 0.0]\n" % (grid.x_min, grid.y_min)
        + "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.19\n",
        encoding="utf-8",
    )
    canvas = Image.fromarray(np.repeat(pgm[:, :, None], 3, axis=2), mode="RGB")
    draw = ImageDraw.Draw(canvas)

    def pixel(point):
        cell = grid.world_to_cell(point[0], point[1])
        if cell is None:
            return None
        return cell[0], grid.height - 1 - cell[1]

    path_pixels = [pixel(point) for point in path]
    path_pixels = [point for point in path_pixels if point is not None]
    if len(path_pixels) > 1:
        draw.line(path_pixels, fill=(20, 150, 35), width=max(1, int(0.08 / grid.resolution)))
    trajectory_pixels = [pixel(point) for point in (trajectory or [])]
    trajectory_pixels = [point for point in trajectory_pixels if point is not None]
    if len(trajectory_pixels) > 1:
        draw.line(trajectory_pixels, fill=(35, 90, 220), width=max(1, int(0.06 / grid.resolution)))
    for point, color in ((start, (35, 100, 230)), (goal, (220, 35, 35))):
        if point is None:
            continue
        p = pixel(point)
        if p is not None:
            radius = max(2, int(radius_m / grid.resolution))
            draw.ellipse((p[0] - radius, p[1] - radius, p[0] + radius, p[1] + radius), fill=color)
    canvas.resize((grid.width * 5, grid.height * 5), Image.Resampling.NEAREST).save(output / "slam_map.png")
    metadata = {
        "frame_id": "map", "resolution_m": grid.resolution,
        "width_cells": grid.width, "height_cells": grid.height,
        "origin_xy_m": [grid.x_min, grid.y_min], "integrated_scans": grid.scan_count,
        "occupied_cells": int(occupied.sum()), "free_cells": int(free.sum()),
        "unknown_cells": int((~occupied & ~free).sum()),
    }
    (output / "slam_map.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    return metadata
