"""栅格地图存取：内置演示地图、ROS map_server 地图（YAML + PGM/PNG）与 JSON 存档。

坐标与行序约定（与 ``host_ui/app/types.py`` 一致）
--------------------------------------------------
* 全局坐标单位米。``GridMap`` 第 0 行是地图**最下方**、第 0 列是最左方，
  单元格 (gx, gy) 中心位于 ``origin + R(origin_yaw) * ((gx+0.5)*res, (gy+0.5)*res)``。
* ROS ``map_server`` 的 PGM/PNG 图像是**第 0 行在最上方**，加载时必须做一次
  上下翻转；这是本模块最容易出错的地方，相关代码集中在 ``_pixels_to_grid``。
* 栅格值只有三种：``UNKNOWN=-1``、``FREE=0``、``OCCUPIED=100``。

线程模型
--------
本模块是纯计算 + 文件 I/O，无全局可变状态。``demo_map()`` 每次调用都重新构建
（约 1 ms），调用方可以安全地在任意线程加载地图后通过回调交给后端。
PNG 解码只用标准库 ``zlib``，不引入 Pillow；不支持的 PNG 子格式抛出中文
``ValueError``，由上层提示用户转成 PGM。

ROS 地图语义
------------
``map_server`` 的 YAML 关键字段：``image``、``resolution``、``origin: [x, y, yaw]``、
``negate``。图像像素按 ``negate`` 转换：非 negate 时 ``occ = (255-p)/255``
（黑=占据、白=空闲），negate 时 ``occ = p/255``；未知区域在非 negate 图像中
是 205、在 negate 图像中是 50。本模块按阈值映射到 0/100，未知映射到 -1。
"""
from __future__ import annotations

import json
import math
import struct
import zlib
from array import array
from pathlib import Path
from typing import Any, Iterable

from .types import FREE, OCCUPIED, UNKNOWN, GridMap

# ---------------------------------------------------------------------------
# 演示地图参数（前端离线演示与 MockTransport 共用）
# ---------------------------------------------------------------------------
#: 演示地图宽（米）
DEMO_WIDTH_M = 18.0
#: 演示地图高（米）
DEMO_HEIGHT_M = 12.0
#: 演示地图分辨率（米/格）
DEMO_RESOLUTION = 0.05
#: 演示地图原点（世界坐标，米/度）；机器人默认位于 (0, 0)，即地图中心
DEMO_ORIGIN = (-9.0, -6.0, 0.0)
#: 演示地图名称
DEMO_NAME = "demo_lab"

#: 默认分辨率（用于没有 YAML 的裸 PGM/PNG）
DEFAULT_RESOLUTION = 0.05


def _paint_cells(grid: GridMap, gx0: int, gy0: int, gx1: int, gy1: int,
                 value: int = OCCUPIED) -> None:
    """按栅格索引闭区间填充矩形（越界自动裁剪）。"""
    gx0, gx1 = sorted((int(gx0), int(gx1)))
    gy0, gy1 = sorted((int(gy0), int(gy1)))
    gx0, gy0 = max(0, gx0), max(0, gy0)
    gx1, gy1 = min(grid.width - 1, gx1), min(grid.height - 1, gy1)
    raw = value & 0xFF
    for gy in range(gy0, gy1 + 1):
        base = gy * grid.width
        for gx in range(gx0, gx1 + 1):
            grid.data[base + gx] = raw


def _paint_world_rect(grid: GridMap, x0: float, y0: float, x1: float, y1: float,
                      value: int = OCCUPIED) -> None:
    """按世界坐标矩形填充（内部换算到栅格索引，边界向外取整）。"""
    ox, oy, _ = grid.origin
    res = grid.resolution
    gx0 = int(math.floor((min(x0, x1) - ox) / res))
    gx1 = int(math.ceil((max(x0, x1) - ox) / res)) - 1
    gy0 = int(math.floor((min(y0, y1) - oy) / res))
    gy1 = int(math.ceil((max(y0, y1) - oy) / res)) - 1
    _paint_cells(grid, gx0, gy0, gx1, gy1, value)


def demo_map() -> GridMap:
    """构建 18 m x 12 m 室内实验室演示地图（360 x 240 格，0.05 m/格）。

    内容：0.2 m 厚外墙、三间由隔断分开的房间（各留 1.2 m 门洞）、若干柱子和
    箱子。机器人默认起点 (0, 0)（地图中心）周围半径 1 m 保持空旷，方便
    离线演示直接前进/转向。完全确定性构建（无随机数），重复调用结果相同。
    """
    width = int(round(DEMO_WIDTH_M / DEMO_RESOLUTION))
    height = int(round(DEMO_HEIGHT_M / DEMO_RESOLUTION))
    # 所有区域先标记为可通行，再画墙与障碍
    grid = GridMap(width, height, DEMO_RESOLUTION, DEMO_ORIGIN,
                   data=bytes([FREE]) * (width * height), name=DEMO_NAME)

    wall = int(round(0.2 / DEMO_RESOLUTION))  # 外墙厚度 4 格
    # ---- 外墙 ---------------------------------------------------------
    _paint_cells(grid, 0, 0, width - 1, wall - 1)                 # 下
    _paint_cells(grid, 0, height - wall, width - 1, height - 1)   # 上
    _paint_cells(grid, 0, 0, wall - 1, height - 1)                # 左
    _paint_cells(grid, width - wall, 0, width - 1, height - 1)    # 右

    half = 0.075  # 内隔断半厚（总厚 0.15 m）

    # ---- 纵向隔断 x=-2.5，把左侧房间与主厅分开，门洞 y ∈ [-1.0, 0.2]
    for y0, y1 in ((-6.0, -1.0), (0.2, 6.0)):
        _paint_world_rect(grid, -2.5 - half, y0, -2.5 + half, y1)

    # ---- 横向隔断 y=1.5，把左侧再分成上下两间，门洞 x ∈ [-6.5, -5.3]
    for x0, x1 in ((-9.0, -6.5), (-5.3, -2.5)):
        _paint_world_rect(grid, x0, 1.5 - half, x1, 1.5 + half)

    # ---- 纵向隔断 x=4.0，右下测试区，门洞 y ∈ [-2.6, -1.4]
    for y0, y1 in ((-6.0, -2.6), (-1.4, 0.5)):
        _paint_world_rect(grid, 4.0 - half, y0, 4.0 + half, y1)

    # ---- 柱子 0.4 m x 0.4 m（主厅与各房间）
    for cx, cy in ((2.0, 2.5), (2.4, -2.2), (6.6, 4.6), (-7.2, 4.0)):
        _paint_world_rect(grid, cx - 0.2, cy - 0.2, cx + 0.2, cy + 0.2)

    # ---- 箱子/器材 (中心 x, 中心 y, 长, 宽)
    for cx, cy, sx, sy in ((6.8, -2.6, 0.6, 0.5), (6.4, 2.4, 0.5, 0.6),
                           (-7.0, -3.8, 0.7, 0.5), (-6.0, 4.8, 0.5, 0.5)):
        _paint_world_rect(grid, cx - sx / 2, cy - sy / 2,
                          cx + sx / 2, cy + sy / 2)

    # 起点空旷是地图契约的一部分，构建时就校验，避免后续改动破坏演示
    if grid.is_occupied_world(0.0, 0.0, 1.0):
        raise RuntimeError("演示地图构建错误: 机器人起点 (0, 0) 半径 1 m 内有障碍")
    return grid


# ---------------------------------------------------------------------------
# ROS map_server 地图加载
# ---------------------------------------------------------------------------
def _parse_ros_map_yaml(text: str, source: str) -> dict[str, Any]:
    """解析 map_server 的 YAML（只取本模块需要的字段，不依赖 PyYAML）。"""
    fields: dict[str, Any] = {}
    for raw_line in text.splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, _, value = line.partition(":")
        key = key.strip().lower()
        value = value.strip()
        if not value:
            continue
        try:
            if key == "image":
                fields["image"] = value.strip("'\"")
            elif key == "resolution":
                fields["resolution"] = float(value)
            elif key == "negate":
                fields["negate"] = value.lower() in ("true", "1", "yes", "on")
            elif key in ("width", "height"):
                fields[key] = int(float(value))
            elif key in ("origin", "start"):
                parts = [part for part in value.strip("[]").split(",") if part.strip()]
                fields[key] = [float(part) for part in parts]
        except ValueError:
            raise ValueError(
                f"地图描述文件 {source} 的 {key} 字段无法解析: {value!r}") from None
    return fields


def _skip_pnm_header(raw: bytes) -> tuple[str, int, int, int, int]:
    """解析 PNM 头部，返回 (magic, width, height, maxval, 光栅起始偏移)。"""
    pos = 0
    tokens: list[bytes] = []
    while len(tokens) < 4:
        while pos < len(raw) and raw[pos:pos + 1].isspace():
            pos += 1
        if pos >= len(raw):
            raise ValueError("PGM 头部不完整")
        if raw[pos:pos + 1] == b"#":
            while pos < len(raw) and raw[pos:pos + 1] not in (b"\n", b"\r"):
                pos += 1
            continue
        start = pos
        while pos < len(raw) and not raw[pos:pos + 1].isspace():
            pos += 1
        if start == pos:
            raise ValueError("PGM 头部不完整")
        tokens.append(raw[start:pos])
    try:
        magic = tokens[0].decode("ascii")
        width = int(tokens[1])
        height = int(tokens[2])
        maxval = int(tokens[3])
    except (UnicodeDecodeError, ValueError):
        raise ValueError("PGM 头部字段无法解析") from None
    # 头部与光栅之间只允许一个空白字符（\r\n 视为一个）
    if raw[pos:pos + 2] == b"\r\n":
        pos += 2
    elif pos < len(raw) and raw[pos:pos + 1].isspace():
        pos += 1
    return magic, width, height, maxval, pos


def _decode_pnm(raw: bytes) -> tuple[int, int, int, list[int]]:
    """解码 P5（二进制）/P2（ASCII）PNM，返回 (宽, 高, maxval, 像素行主序)。"""
    magic, width, height, maxval, pos = _skip_pnm_header(raw)
    if magic not in ("P2", "P5"):
        raise ValueError(f"不支持的 PNM 格式 {magic}，只支持 P2/P5（PGM）")
    if width <= 0 or height <= 0:
        raise ValueError("PGM 宽高必须为正")
    if not 0 < maxval <= 65535:
        raise ValueError(f"PGM maxval 非法: {maxval}")
    count = width * height
    if magic == "P5":
        if maxval < 256:
            raster = raw[pos:pos + count]
            if len(raster) != count:
                raise ValueError("PGM 像素数据长度不足")
            return width, height, maxval, list(raster)
        raster = raw[pos:pos + count * 2]
        if len(raster) != count * 2:
            raise ValueError("PGM 像素数据长度不足")
        values = array("H")
        values.frombytes(raster)
        if values.itemsize != 2:  # pragma: no cover - 仅理论平台
            raise ValueError("当前平台不支持 16 位 PGM")
        # PNM 规定 16 位样本为大端；array 按本机字节序解释，这里统一转成大端
        if array("H", [1]).tobytes() == b"\x01\x00":
            values.byteswap()
        return width, height, maxval, list(values)
    # P2：ASCII 十进制
    try:
        values = [int(token) for token in raw[pos:].split()]
    except ValueError:
        raise ValueError("PGM(P2) 像素数据含非整数值") from None
    if len(values) < count:
        raise ValueError("PGM(P2) 像素数量不足")
    return width, height, maxval, values[:count]


def _unfilter_png_line(kind: int, line: bytearray, prev: bytearray, bpp: int) -> None:
    """对 PNG 单行做逆过滤（filter 0~4）。"""
    if kind == 0:
        return
    if kind == 1:  # Sub
        for i in range(bpp, len(line)):
            line[i] = (line[i] + line[i - bpp]) & 0xFF
    elif kind == 2:  # Up
        for i in range(len(line)):
            line[i] = (line[i] + prev[i]) & 0xFF
    elif kind == 3:  # Average
        for i in range(len(line)):
            left = line[i - bpp] if i >= bpp else 0
            line[i] = (line[i] + ((left + prev[i]) >> 1)) & 0xFF
    elif kind == 4:  # Paeth
        for i in range(len(line)):
            a = line[i - bpp] if i >= bpp else 0
            b = prev[i]
            c = prev[i - bpp] if i >= bpp else 0
            p = a + b - c
            pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
            predictor = a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
            line[i] = (line[i] + predictor) & 0xFF
    else:
        raise ValueError(f"不支持的 PNG 行过滤类型: {kind}")


def _decode_png(raw: bytes) -> tuple[int, int, int, list[int]]:
    """解码 8 位非隔行 PNG（灰度/RGB/RGBA），返回 (宽, 高, 255, 像素)。"""
    if not raw.startswith(b"\x89PNG\r\n\x1a\n"):
        raise ValueError("不是有效的 PNG 文件")
    pos = 8
    width = height = bitdepth = colortype = interlace = None
    idat = bytearray()
    while pos + 8 <= len(raw):
        length = struct.unpack_from(">I", raw, pos)[0]
        tag = raw[pos + 4:pos + 8]
        payload = raw[pos + 8:pos + 8 + length]
        pos += 12 + length
        if tag == b"IHDR":
            (width, height, bitdepth, colortype,
             _compression, _filtering, interlace) = struct.unpack(">IIBBBBB", payload)
        elif tag == b"IDAT":
            idat += payload
        elif tag == b"IEND":
            break
    if width is None or height is None:
        raise ValueError("PNG 缺少 IHDR 块")
    if bitdepth != 8 or interlace != 0 or colortype not in (0, 2, 4, 6):
        raise ValueError(
            f"暂不支持该 PNG 格式（位深={bitdepth}, 颜色类型={colortype}, "
            f"隔行={interlace}）；请转换为 8 位灰度 PGM 后加载")
    channels = {0: 1, 2: 3, 4: 2, 6: 4}[colortype]
    stride = width * channels
    try:
        raster = zlib.decompress(bytes(idat))
    except zlib.error as error:
        raise ValueError(f"PNG 数据解压失败: {error}") from None
    if len(raster) < height * (stride + 1):
        raise ValueError("PNG 像素数据长度不足")
    values: list[int] = []
    prev = bytearray(stride)
    index = 0
    for _ in range(height):
        kind = raster[index]
        index += 1
        line = bytearray(raster[index:index + stride])
        index += stride
        _unfilter_png_line(kind, line, prev, channels)
        # 灰度取唯一通道；彩色取 R 通道做亮度近似（地图为黑/白/灰，足够）
        values.extend(line[0::channels])
        prev = line
    return width, height, 255, values


def _load_image(path: Path) -> tuple[int, int, int, list[int]]:
    """按文件内容（而非扩展名）解码 PGM/PNG，返回 (宽, 高, maxval, 像素)。"""
    raw = path.read_bytes()
    if raw.startswith(b"P5") or raw.startswith(b"P2"):
        return _decode_pnm(raw)
    if raw.startswith(b"\x89PNG\r\n\x1a\n"):
        return _decode_png(raw)
    raise ValueError(f"不支持的图像格式: {path}（只支持 PGM 的 P5/P2 或 8 位非隔行 PNG）")


def _pixel_to_cell(pixel_255: int, negate: bool) -> int:
    """把 8 位灰度像素映射为 GridMap 取值：0 空闲、100 占据、-1 未知。"""
    unknown_pixel = 50 if negate else 205
    if abs(pixel_255 - unknown_pixel) <= 2:
        return UNKNOWN
    occupancy = pixel_255 / 255.0 if negate else (255.0 - pixel_255) / 255.0
    if occupancy > 0.65:
        return OCCUPIED
    if occupancy < 0.196:
        return FREE
    return UNKNOWN


def _pixels_to_grid(width: int, height: int, maxval: int, pixels: Iterable[int],
                    resolution: float, origin: tuple[float, float, float],
                    negate: bool, name: str) -> GridMap:
    """PNM/PNG 像素（第 0 行在最上方）转 GridMap（第 0 行在最下方）。"""
    if width <= 0 or height <= 0:
        raise ValueError("地图图像宽高必须为正")
    values = list(pixels)
    count = width * height
    if len(values) < count:
        raise ValueError(f"地图图像像素数量不足: 需要 {count}，实际 {len(values)}")
    cells = bytearray(count)
    scale = 255.0 / maxval if maxval != 255 else None
    for row in range(height):
        target_gy = height - 1 - row  # 行翻转：图像顶部 -> 地图最上方
        source_base = row * width
        target_base = target_gy * width
        for col in range(width):
            pixel = values[source_base + col]
            pixel_255 = int(round(pixel * scale)) if scale is not None else pixel
            cells[target_base + col] = _pixel_to_cell(pixel_255, negate) & 0xFF
    return GridMap(width, height, resolution, origin, data=bytes(cells), name=name)


def load_ros_map(yaml_path: str | Path) -> GridMap:
    """加载 ROS ``map_server`` 地图（YAML + PGM/PNG）。

    支持 ``resolution`` / ``origin: [x, y, yaw]`` / ``negate``，图像支持
    P5、P2（maxval 255 或 65535）和 8 位非隔行 PNG。图像行序会翻转成
    GridMap 约定（第 0 行最下方），未知像素（205，negate 时为 50）映射为 -1。
    """
    path = Path(yaml_path)
    if not path.is_file():
        raise ValueError(f"地图描述文件不存在: {path}")
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError as error:
        raise ValueError(f"读取地图描述文件失败: {path}: {error}") from None
    fields = _parse_ros_map_yaml(text, str(path))
    if "image" not in fields:
        raise ValueError(f"地图描述文件缺少 image 字段: {path}")
    image = Path(str(fields["image"]))
    if not image.is_absolute():
        image = path.parent / image
    if not image.is_file():
        raise ValueError(f"地图描述文件引用的图像不存在: {image}")
    resolution = float(fields.get("resolution", DEFAULT_RESOLUTION))
    if not (resolution > 0):
        raise ValueError(f"地图分辨率必须为正: {resolution}")
    origin_values = fields.get("origin") or [0.0, 0.0, 0.0]
    origin = (
        float(origin_values[0]) if len(origin_values) > 0 else 0.0,
        float(origin_values[1]) if len(origin_values) > 1 else 0.0,
        float(origin_values[2]) if len(origin_values) > 2 else 0.0,
    )
    negate = bool(fields.get("negate", False))
    width, height, maxval, pixels = _load_image(image)
    return _pixels_to_grid(width, height, maxval, pixels, resolution, origin,
                           negate, path.stem)


def load_image_map(image_path: str | Path, resolution: float = DEFAULT_RESOLUTION,
                   origin: tuple[float, float, float] = (0.0, 0.0, 0.0),
                   negate: bool = False, name: str | None = None) -> GridMap:
    """加载裸 PGM/PNG（没有 YAML 元数据时使用默认分辨率与原点）。"""
    path = Path(image_path)
    if not path.is_file():
        raise ValueError(f"地图图像不存在: {path}")
    width, height, maxval, pixels = _load_image(path)
    return _pixels_to_grid(width, height, maxval, pixels, float(resolution),
                           (float(origin[0]), float(origin[1]),
                            float(origin[2]) if len(origin) > 2 else 0.0),
                           bool(negate), name or path.stem)


def map_from_occupancy_grid(width: int, height: int, resolution: float,
                            origin: Iterable[float],
                            data: Iterable[int], name: str = "map") -> GridMap:
    """把 ROS ``nav_msgs/OccupancyGrid`` 的字段转成 :class:`GridMap`。

    纯函数，不依赖 ROS 消息类型，便于离线单测：``data`` 直接传
    ``OccupancyGrid.data``（-1 未知，0~100 占据概率），``origin`` 传
    ``[origin.position.x, origin.position.y, origin_yaw_deg]``。
    ROS 的 OccupancyGrid 行序与 GridMap 一致（第 0 行最下方），无需翻转。
    """
    width, height = int(width), int(height)
    if width <= 0 or height <= 0:
        raise ValueError("占据栅格宽高必须为正")
    origin_values = list(origin)
    if len(origin_values) < 2:
        raise ValueError("占据栅格原点至少需要 x、y")
    origin_tuple = (
        float(origin_values[0]),
        float(origin_values[1]),
        float(origin_values[2]) if len(origin_values) > 2 else 0.0,
    )
    values = list(data)
    if len(values) != width * height:
        raise ValueError(
            f"占据栅格数据长度 {len(values)} 与 {width}x{height} 不匹配")
    cells = bytearray(width * height)
    for index, value in enumerate(values):
        try:
            level = int(value)
        except (TypeError, ValueError):
            raise ValueError(f"占据栅格数据第 {index} 个值不是整数: {value!r}") from None
        if level < 0:
            cells[index] = UNKNOWN & 0xFF
        elif level >= 50:
            cells[index] = OCCUPIED & 0xFF
        else:
            cells[index] = FREE
    return GridMap(width, height, float(resolution), origin_tuple,
                   data=bytes(cells), name=name)


# ---------------------------------------------------------------------------
# 通用分发与 JSON 存档
# ---------------------------------------------------------------------------
def load_map_file(path: str | Path) -> GridMap:
    """按扩展名加载地图文件。

    * ``.yaml/.yml`` -> :func:`load_ros_map`
    * ``.json`` -> :func:`load_map_payload`（本模块自己的存档格式）
    * ``.pgm/.pnm/.png`` -> 若存在同名 YAML 则按 ROS 地图加载，否则用默认分辨率
    * 其它类型抛 ``ValueError``（中文提示支持的格式）
    """
    file_path = Path(path)
    if not file_path.is_file():
        raise ValueError(f"地图文件不存在: {file_path}")
    suffix = file_path.suffix.lower()
    if suffix in (".yaml", ".yml"):
        return load_ros_map(file_path)
    if suffix == ".json":
        return load_map_payload(file_path)
    if suffix in (".pgm", ".pnm", ".png"):
        sibling_yaml = file_path.with_suffix(".yaml")
        if sibling_yaml.is_file():
            return load_ros_map(sibling_yaml)
        return load_image_map(file_path)
    raise ValueError(
        f"不支持的地图文件类型: {suffix or '（无扩展名）'}；"
        f"支持 .yaml/.yml（ROS map_server）、.json（本机存档）、.pgm/.pnm/.png")


def save_map_payload(grid: GridMap, path: str | Path) -> None:
    """把地图保存成本模块的 JSON 存档（内容即 ``grid.to_payload()``）。"""
    file_path = Path(path)
    if file_path.parent and not file_path.parent.exists():
        file_path.parent.mkdir(parents=True, exist_ok=True)
    payload = grid.to_payload()
    try:
        with file_path.open("w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=1)
    except OSError as error:
        raise ValueError(f"保存地图失败: {file_path}: {error}") from None


def load_map_payload(path: str | Path) -> GridMap:
    """读取 :func:`save_map_payload` 写出的 JSON 存档。"""
    file_path = Path(path)
    if not file_path.is_file():
        raise ValueError(f"地图存档不存在: {file_path}")
    try:
        payload = json.loads(file_path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise ValueError(f"地图存档无法解析: {file_path}: {error}") from None
    if not isinstance(payload, dict):
        raise ValueError(f"地图存档格式错误: {file_path} 顶层必须是对象")
    try:
        return GridMap.from_payload(payload)
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"地图存档字段不完整或非法: {file_path}: {error}") from None


__all__ = [
    "DEFAULT_RESOLUTION",
    "DEMO_HEIGHT_M",
    "DEMO_NAME",
    "DEMO_ORIGIN",
    "DEMO_RESOLUTION",
    "DEMO_WIDTH_M",
    "demo_map",
    "load_image_map",
    "load_map_file",
    "load_map_payload",
    "load_ros_map",
    "map_from_occupancy_grid",
    "save_map_payload",
]