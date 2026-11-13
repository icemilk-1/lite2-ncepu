"""Guarded ROS 1 Twist publisher for the Lite2 vendor velocity bridge.

ROS and geometry_msgs are injected by the hardware-side node so the portable
navigation package can be imported and tested without a ROS installation.
"""


class Ros1TwistExecutionAdapter(object):
    def __init__(self, publisher, twist_factory, topic, max_linear_x_m_s,
                 max_angular_z_rad_s, watchdog_timeout_s,
                 single_writer_verified=False, mode_ready=None):
        if not single_writer_verified:
            raise RuntimeError("ROS velocity publishing requires verified single-writer ownership.")
        if mode_ready is None:
            raise RuntimeError("A vendor motion-mode readiness check is required.")
        if not topic:
            raise ValueError("A ROS output topic must be configured.")
        self.publisher = publisher
        self.twist_factory = twist_factory
        self.topic = str(topic)
        self.max_linear_x_m_s = abs(float(max_linear_x_m_s))
        self.max_angular_z_rad_s = abs(float(max_angular_z_rad_s))
        self.watchdog_timeout_s = float(watchdog_timeout_s)
        if self.watchdog_timeout_s <= 0.0:
            raise ValueError("The command watchdog timeout must be positive.")
        self.mode_ready = mode_ready
        self.cancelled = False
        self.deadline_s = None
        self.stop_latched = False

    def publish(self, command, now_s):
        now_s = float(now_s)
        try:
            ready = bool(self.mode_ready())
        except Exception:
            ready = False
        if (self.cancelled or not ready or command.expired(now_s)
                or command.created_at_s > now_s):
            self.stop(now_s)
            return False
        message = self.twist_factory()
        message.linear.x = self._clamp(command.linear_x_m_s, self.max_linear_x_m_s)
        message.linear.y = 0.0
        message.angular.z = self._clamp(command.angular_z_rad_s, self.max_angular_z_rad_s)
        self.publisher.publish(message)
        self.deadline_s = min(command.expires_at_s, now_s + self.watchdog_timeout_s)
        self.stop_latched = False
        return True

    def watchdog(self, now_s):
        if self.cancelled:
            return False
        if self.deadline_s is not None and float(now_s) >= self.deadline_s:
            self.stop(now_s)
            return True
        return False

    def cancel(self, now_s):
        self.cancelled = True
        self.stop(now_s)

    def stop(self, now_s):
        if not self.stop_latched:
            message = self.twist_factory()
            message.linear.x = 0.0
            message.linear.y = 0.0
            message.angular.z = 0.0
            self.publisher.publish(message)
            self.stop_latched = True
        self.deadline_s = None

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, float(value)))
