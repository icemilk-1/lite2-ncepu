"""Pure command validation shared by the ROS bridge and regression tests."""
import math

MAX_FORWARD_M_S = 1.0
MAX_REVERSE_M_S = 1.0
MAX_YAW_RAD_S = 1.0


def fresh(timestamp, now, timeout):
    return (isinstance(timestamp, (int, float)) and math.isfinite(timestamp)
            and 0.0 <= now - timestamp <= timeout)


def guarded_velocity(request, now, state, armed, mode_confirmed, depth_time,
                     limits=None):
    linear, angular, created = request
    reason = None
    if not armed:
        reason = "disarmed"
    elif not mode_confirmed:
        reason = "mode_unconfirmed"
    elif not fresh(state.get("received"), now, 0.3):
        reason = "stale_state"
    elif not fresh(state.get("command_received"), now, 0.3):
        reason = "stale_command_feedback"
    elif not fresh(depth_time, now, 0.5):
        reason = "stale_depth"
    elif state.get("basic") != 6:
        reason = "motion_mode_not_ready"
    elif state.get("error_state", 0):
        reason = "vendor_error"
    elif max(abs(v) for v in state.get("joystick", [1.0])) >= 0.03:
        reason = "remote_takeover"
    elif max(abs(v) for v in state.get("rpy_deg", [180])[:2]) > 18:
        reason = "body_tilt"
    elif not fresh(created, now, 0.25):
        reason = "command_timeout"
    elif not all(math.isfinite(v) for v in (linear, angular)):
        reason = "invalid_command"
    if reason:
        return 0.0, 0.0, reason
    configured = limits if limits is not None else {}
    try:
        forward = float(configured.get("max_forward_m_s", MAX_FORWARD_M_S))
        reverse = float(configured.get("max_reverse_m_s", MAX_REVERSE_M_S))
        yaw = float(configured.get("max_yaw_rad_s", MAX_YAW_RAD_S))
    except (AttributeError, TypeError, ValueError, OverflowError):
        return 0.0, 0.0, "invalid_limits"
    if not all(math.isfinite(value) and value >= 0 for value in (forward, reverse, yaw)):
        return 0.0, 0.0, "invalid_limits"
    return (max(-reverse, min(forward, linear)),
            max(-yaw, min(yaw, angular)), "enabled")
