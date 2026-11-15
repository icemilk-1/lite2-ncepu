"""Lite2 V1.2.3 packet layouts, with four-byte wire alignment."""
import math
import struct
import re


def decode_state(packet):
    if len(packet) != 196 or struct.unpack_from("<3i", packet) != (2305, 184, 1):
        return None
    basic, gait = struct.unpack_from("<2i", packet, 12)
    values = struct.unpack_from("<18d", packet, 20)
    if (not all(math.isfinite(value) for value in values)
            or max(abs(value) for value in values[:3]) > 180
            or max(abs(value) for value in values[12:18]) > 5):
        return None
    return dict(basic=basic, gait=gait,
                motion=struct.unpack_from("<i", packet, 176)[0],
                error_state=struct.unpack_from("<I", packet, 172)[0],
                battery_level_raw=struct.unpack_from("<d", packet, 180)[0],
                rpy_deg=list(values[0:3]), angular=list(values[3:6]),
                position=list(values[9:12]), velocity_world=list(values[12:15]),
                velocity_body=list(values[15:18]))


def velocity_packets(linear, angular):
    return [struct.pack("<3id", code, 8, 1, value)
            for code, value in ((320, linear), (325, 0.0), (321, angular))]


def decode_command(packet):
    if len(packet) != 60 or struct.unpack_from("<3i", packet) != (2309, 48, 1):
        return None
    values = struct.unpack_from("<6d", packet, 12)
    if not all(math.isfinite(value) for value in values):
        return None
    return dict(joystick=list(values[:3]), goal_velocity=list(values[3:]))


def latest_mode_ack(log_text):
    """Read actual received mode packets printed by the factory UDP bridge."""
    last = None
    for line in log_text.splitlines():
        match = re.fullmatch(r"\((553716738|553716739), 0, 0\)", line.strip())
        if match:
            last = int(match.group(1))
    return last
