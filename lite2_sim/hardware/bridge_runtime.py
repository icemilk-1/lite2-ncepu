"""Process ownership and durable control handoff for the ROS vendor bridge (Python 3.6)."""
import errno
import json
import os
import socket
import tempfile


class ControlOwnerStore:
    """Preserve explicit handoff across bridge restarts; a new installation owns standby."""

    MODES = ("host", "release", "estop", "remote_takeover")

    def __init__(self, path=None):
        self.path = path or os.environ.get(
            "AIRACE_CONTROL_OWNER_FILE",
            os.path.expanduser("~/.local/state/airace/control_owner.json"))

    def load(self):
        try:
            with open(self.path, "r") as source:
                mode = json.load(source)["mode"]
        except OSError as error:
            if error.errno == errno.ENOENT:
                return "host"
            raise
        except (ValueError, KeyError, TypeError) as error:
            raise RuntimeError("Invalid control owner state: %s" % error)
        if mode not in self.MODES:
            raise RuntimeError("Unknown control owner: %s" % mode)
        return mode

    def save(self, mode):
        if mode not in self.MODES:
            raise ValueError("Unknown control owner: %s" % mode)
        directory = os.path.dirname(os.path.abspath(self.path))
        os.makedirs(directory, mode=0o700, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(prefix=".control_owner_", dir=directory)
        try:
            with os.fdopen(descriptor, "w") as target:
                json.dump({"mode": mode}, target)
                target.flush()
                os.fsync(target.fileno())
            os.replace(temporary, self.path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)


def open_bridge_sockets(receive_port=43898, source_port=43893):
    """Own both UDP ports before the fixed ROS node name can replace an existing node."""
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        receiver.bind(("127.0.0.1", receive_port))
        receiver.settimeout(0.05)
        # No SO_REUSEADDR and no ephemeral fallback: conflicts must fail closed.
        sender.bind(("0.0.0.0", source_port))
        return receiver, sender
    except Exception:
        receiver.close()
        sender.close()
        raise
