"""地图模块测试：演示地图、ROS YAML + PGM/PNG 加载、JSON 存档、OccupancyGrid 转换。"""
import json
import math
import struct
import zlib

import pytest

from host_ui.app.mapstore import (
    DEMO_ORIGIN,
    DEMO_RESOLUTION,
    demo_map,
    load_image_map,
    load_map_file,
    load_map_payload,
    load_ros_map,
    map_from_occupancy_grid,
    save_map_payload,
)
from host_ui.app.types import FREE, OCCUPIED, UNKNOWN, GridMap


# ---------------------------------------------------------------------------
# 测试用地图文件构造
# ---------------------------------------------------------------------------
def write_pgm_p5(path, rows, maxval=255):
    height = len(rows)
    width = len(rows[0])
    assert all(len(row) == width for row in rows)
    header = f"P5\n{width} {height}\n{maxval}\n".encode("ascii")
    if maxval < 256:
        raster = bytes(value for row in rows for value in row)
    else:
        raster = b"".join(struct.pack(">H", value) for row in rows for value in row)
    path.write_bytes(header + raster)


def write_pgm_p2(path, rows, maxval=255):
    height = len(rows)
    width = len(rows[0])
    body = "\n".join(" ".join(str(value) for value in row) for row in rows)
    path.write_text(f"P2\n{width} {height}\n{maxval}\n{body}\n", encoding="ascii")


def write_map_yaml(path, image_name, resolution=0.05, origin=(0.0, 0.0, 0.0),
                   negate=False, comment=False):
    lines = []
    if comment:
        lines.append("# 测试用 map_server 描述")
    lines.append(f"image: {image_name}")
    lines.append(f"resolution: {resolution}")
    lines.append(f"origin: [{origin[0]}, {origin[1]}, {origin[2]}]")
    lines.append(f"negate: {'true' if negate else 'false'}")
    lines.append("occupied_thresh: 0.65")
    lines.append("free_thresh: 0.196")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_gray_png(path, rows):
    height = len(rows)
    width = len(rows[0])
    raw = b"".join(b"\x00" + bytes(row) for row in rows)
    idat = zlib.compress(raw)

    def chunk(tag, payload):
        body = tag + payload
        return (struct.pack(">I", len(payload)) + body
                + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF))

    signature = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    path.write_bytes(signature + chunk(b"IHDR", ihdr)
                     + chunk(b"IDAT", idat) + chunk(b"IEND", b""))


# ---------------------------------------------------------------------------
# 演示地图
# ---------------------------------------------------------------------------
def test_demo_map_shape_and_metadata():
    grid = demo_map()
    assert isinstance(grid, GridMap)
    assert (grid.width, grid.height) == (360, 240)
    assert grid.resolution == pytest.approx(DEMO_RESOLUTION)
    assert grid.origin == pytest.approx(DEMO_ORIGIN)
    assert grid.name == "demo_lab"
    assert grid.cells == 360 * 240


def test_demo_map_is_deterministic():
    first = demo_map()
    second = demo_map()
    assert bytes(first.data) == bytes(second.data)


def test_demo_map_walls_and_rooms():
    grid = demo_map()
    # 外墙四条边都必须是占据
    assert grid.at(0, 0) == OCCUPIED
    assert grid.at(grid.width - 1, grid.height - 1) == OCCUPIED
    assert grid.at(180, 0) == OCCUPIED
    assert grid.at(0, 120) == OCCUPIED
    # 内部隔断 x=-2.5 处占据，门洞 y ∈ [-1.0, 0.2] 处可通行
    assert grid.is_occupied_world(-2.5, -3.0)
    assert grid.is_occupied_world(-2.5, 3.0)
    assert not grid.is_occupied_world(-2.5, -0.4)
    # 横向隔断 y=1.5 有门洞 x ∈ [-6.5, -5.3]
    assert grid.is_occupied_world(-8.0, 1.5)
    assert not grid.is_occupied_world(-5.9, 1.5)
    # 障碍物（柱子/箱子）必须存在，且地图不能几乎全空
    assert grid.is_occupied_world(2.0, 2.5)
    occupied = sum(1 for value in grid.data if value >= OCCUPIED)
    assert occupied > 500


def test_demo_map_start_area_is_clear():
    grid = demo_map()
    assert not grid.is_occupied_world(0.0, 0.0, 1.0)
    assert not grid.is_occupied_world(0.0, 0.0, 0.0)
    assert grid.at(*grid.world_to_index(0.0, 0.0)) == FREE


# ---------------------------------------------------------------------------
# ROS map_server 地图
# ---------------------------------------------------------------------------
def test_load_ros_map_p5_flips_rows_and_maps_values(tmp_path):
    image = tmp_path / "map.pgm"
    # 图像顶部行 -> GridMap 最上方（gy=1）；底部行 -> gy=0
    write_pgm_p5(image, [[254, 254, 0], [0, 254, 205]])
    yaml_path = tmp_path / "map.yaml"
    write_map_yaml(yaml_path, "map.pgm", resolution=0.1,
                   origin=(-1.0, -2.0, 90.0), comment=True)

    grid = load_ros_map(yaml_path)
    assert (grid.width, grid.height) == (3, 2)
    assert grid.resolution == pytest.approx(0.1)
    assert grid.origin == pytest.approx((-1.0, -2.0, 90.0))
    assert grid.name == "map"
    # 第 0 行（底部）来自图像最后一行
    assert [grid.at(gx, 0) for gx in range(3)] == [OCCUPIED, FREE, UNKNOWN]
    # 第 1 行（顶部）来自图像第一行
    assert [grid.at(gx, 1) for gx in range(3)] == [FREE, FREE, OCCUPIED]


def test_load_ros_map_p2_ascii(tmp_path):
    image = tmp_path / "ascii.pgm"
    write_pgm_p2(image, [[0, 254], [205, 100]])
    yaml_path = tmp_path / "ascii.yaml"
    write_map_yaml(yaml_path, "ascii.pgm")
    grid = load_ros_map(yaml_path)
    assert [grid.at(gx, 0) for gx in range(2)] == [UNKNOWN, UNKNOWN]
    assert [grid.at(gx, 1) for gx in range(2)] == [OCCUPIED, FREE]


def test_load_ros_map_16bit_pgm(tmp_path):
    image = tmp_path / "deep.pgm"
    write_pgm_p5(image, [[0, 65535, 205 * 257]], maxval=65535)
    yaml_path = tmp_path / "deep.yaml"
    write_map_yaml(yaml_path, "deep.pgm")
    grid = load_ros_map(yaml_path)
    assert [grid.at(gx, 0) for gx in range(3)] == [OCCUPIED, FREE, UNKNOWN]


def test_load_ros_map_negate(tmp_path):
    image = tmp_path / "neg.pgm"
    # negate 时：0=空闲、255=占据、50=未知
    write_pgm_p5(image, [[0, 50, 255]])
    yaml_path = tmp_path / "neg.yaml"
    write_map_yaml(yaml_path, "neg.pgm", negate=True)
    grid = load_ros_map(yaml_path)
    assert [grid.at(gx, 0) for gx in range(3)] == [FREE, UNKNOWN, OCCUPIED]


def test_load_ros_map_missing_image_raises(tmp_path):
    yaml_path = tmp_path / "broken.yaml"
    write_map_yaml(yaml_path, "not_there.pgm")
    with pytest.raises(ValueError) as error:
        load_ros_map(yaml_path)
    assert "不存在" in str(error.value)


def test_load_ros_map_png(tmp_path):
    image = tmp_path / "scan.png"
    write_gray_png(image, [[254, 254, 0], [0, 254, 205]])
    yaml_path = tmp_path / "scan.yaml"
    write_map_yaml(yaml_path, "scan.png")
    grid = load_ros_map(yaml_path)
    assert (grid.width, grid.height) == (3, 2)
    assert [grid.at(gx, 0) for gx in range(3)] == [OCCUPIED, FREE, UNKNOWN]
    assert [grid.at(gx, 1) for gx in range(3)] == [FREE, FREE, OCCUPIED]


def test_load_image_map_defaults(tmp_path):
    image = tmp_path / "plain.pgm"
    write_pgm_p5(image, [[0, 254]])
    grid = load_image_map(image)
    assert grid.resolution == pytest.approx(0.05)
    assert grid.origin == pytest.approx((0.0, 0.0, 0.0))
    assert grid.name == "plain"


# ---------------------------------------------------------------------------
# load_map_file 分发
# ---------------------------------------------------------------------------
def test_load_map_file_dispatch(tmp_path):
    image = tmp_path / "lab.pgm"
    write_pgm_p5(image, [[0, 254]])
    yaml_sibling = tmp_path / "lab.yaml"
    write_map_yaml(yaml_sibling, "lab.pgm", resolution=0.2,
                   origin=(3.0, 4.0, 0.0))

    # 有同名 YAML 时使用 YAML 的元数据
    from_yaml = load_map_file(yaml_sibling)
    assert from_yaml.resolution == pytest.approx(0.2)
    assert from_yaml.origin == pytest.approx((3.0, 4.0, 0.0))
    from_pgm = load_map_file(image)
    assert from_pgm.resolution == pytest.approx(0.2)

    # JSON 存档
    archive = tmp_path / "saved.json"
    save_map_payload(from_yaml, archive)
    reloaded = load_map_file(archive)
    assert bytes(reloaded.data) == bytes(from_yaml.data)
    assert reloaded.origin == pytest.approx(from_yaml.origin)

    # 无 YAML 的裸 PGM 用默认分辨率
    lone = tmp_path / "lone.pgm"
    write_pgm_p5(lone, [[254, 254]])
    alone = load_map_file(lone)
    assert alone.resolution == pytest.approx(0.05)

    # 不支持的类型给出中文提示
    unsupported = tmp_path / "map.txt"
    unsupported.write_text("x", encoding="utf-8")
    with pytest.raises(ValueError) as error:
        load_map_file(unsupported)
    assert "不支持" in str(error.value)


def test_map_payload_roundtrip(tmp_path):
    grid = GridMap(4, 3, 0.2, (1.0, 2.0, 30.0),
                   data=bytes(value & 0xFF for value in
                              ([FREE, FREE, OCCUPIED, UNKNOWN] * 3)),
                   name="roundtrip")
    path = tmp_path / "archive" / "nested.json"  # 父目录不存在时应自动创建
    save_map_payload(grid, path)
    loaded = load_map_payload(path)
    assert (loaded.width, loaded.height) == (4, 3)
    assert loaded.resolution == pytest.approx(0.2)
    assert loaded.origin == pytest.approx((1.0, 2.0, 30.0))
    assert loaded.name == "roundtrip"
    assert [loaded.at(gx, 0) for gx in range(4)] == [FREE, FREE, OCCUPIED, UNKNOWN]
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["row_order"] == "bottom"


# ---------------------------------------------------------------------------
# ROS OccupancyGrid -> GridMap
# ---------------------------------------------------------------------------
def test_map_from_occupancy_grid():
    grid = map_from_occupancy_grid(2, 2, 0.1, (1.0, 2.0, 45.0),
                                   [-1, 0, 50, 100], name="ros_map")
    assert (grid.width, grid.height) == (2, 2)
    assert grid.resolution == pytest.approx(0.1)
    assert grid.origin == pytest.approx((1.0, 2.0, 45.0))
    assert grid.name == "ros_map"
    assert [grid.at(gx, gy) for gy in range(2) for gx in range(2)] == [
        UNKNOWN, FREE, OCCUPIED, OCCUPIED]


def test_map_from_occupancy_grid_rejects_bad_input():
    with pytest.raises(ValueError):
        map_from_occupancy_grid(3, 3, 0.05, (0.0, 0.0, 0.0), [0, 0, 0])
    with pytest.raises(ValueError):
        map_from_occupancy_grid(1, 1, 0.05, (0.0, 0.0, 0.0), ["x"])


def test_grid_at_handles_unknown_encoding():
    grid = demo_map()
    assert grid.at(-1, -1) == UNKNOWN
    assert grid.at(grid.width + 5, 0) == UNKNOWN
    # 内部存的是无符号字节，at() 必须能还原出 -1（用一张全未知地图验证）。
    # 注：types.GridMap(data=None) 在 Python 3 下会因 bytearray([-1]) 抛
    # ValueError，属于已锁定契约的缺陷，这里显式传入 0xFF 字节规避。
    unknown_grid = GridMap(2, 2, 0.05, data=bytes([UNKNOWN & 0xFF]) * 4)
    assert [unknown_grid.at(gx, gy) for gy in range(2) for gx in range(2)] == [
        UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN]
    assert not math.isnan(grid.resolution)