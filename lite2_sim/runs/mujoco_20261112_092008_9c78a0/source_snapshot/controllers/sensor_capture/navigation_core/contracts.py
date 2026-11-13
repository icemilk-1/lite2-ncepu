"""Python 3.6 compatible data contracts for navigation adapters."""

import math


class NavigationObservation(object):
    """Timestamped sensor data in an explicit coordinate frame."""

    __slots__ = ("timestamp_s", "frame_id", "points", "valid", "source", "metadata")

    def __init__(self, timestamp_s, frame_id, points, valid, source, metadata=None):
        self.timestamp_s = float(timestamp_s)
        self.frame_id = str(frame_id)
        self.points = tuple((float(x), float(y), bool(hit)) for x, y, hit in points)
        self.valid = bool(valid)
        self.source = str(source)
        self.metadata = dict(metadata or {})
        if not math.isfinite(self.timestamp_s):
            raise ValueError("Observation timestamp must be finite.")
        if not self.frame_id:
            raise ValueError("Observation frame_id must not be empty.")


class VelocityCommand(object):
    """Velocity-level command with an explicit expiry time."""

    __slots__ = ("linear_x_m_s", "angular_z_rad_s", "created_at_s", "expires_at_s", "source")

    def __init__(self, linear_x_m_s, angular_z_rad_s, created_at_s, expires_at_s, source="navigation_core"):
        self.linear_x_m_s = float(linear_x_m_s)
        self.angular_z_rad_s = float(angular_z_rad_s)
        self.created_at_s = float(created_at_s)
        self.expires_at_s = float(expires_at_s)
        self.source = str(source)
        values = (self.linear_x_m_s, self.angular_z_rad_s, self.created_at_s, self.expires_at_s)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Velocity command fields must be finite.")
        if self.expires_at_s < self.created_at_s:
            raise ValueError("Velocity command expiry precedes its creation time.")

    def expired(self, now_s):
        return float(now_s) > self.expires_at_s

    @property
    def is_zero(self):
        return abs(self.linear_x_m_s) < 1e-9 and abs(self.angular_z_rad_s) < 1e-9
