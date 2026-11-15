"""Verify ROS master responses and the files needed by the onboard mapping runtime."""
import ast
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import threading
from types import SimpleNamespace
import zipfile

import pytest


ROOT = Path(__file__).resolve().parents[2]
STREAM = ROOT / "host_ui/app/transports/ssh_ros_stream.py"


def load_driver_probe(response=None, error=None):
    """Run the production probe with ROS's XML-RPC wrapper replaced locally."""
    tree = ast.parse(STREAM.read_text(encoding="utf-8"))
    function = next(node for node in tree.body
                    if isinstance(node, ast.FunctionDef) and node.name == "lidar_driver_running")

    def get_system_state():
        if error is not None:
            raise error
        return response

    scope = {
        "LIDAR_TOPIC": "/rslidar_points",
        "rospy": SimpleNamespace(get_master=lambda: SimpleNamespace(
            getSystemState=get_system_state)),
    }
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(STREAM), "exec"), scope)
    return scope["lidar_driver_running"]


def test_driver_probe_reads_ros_xmlrpc_publisher_response():
    response = [1, "system state", [[("/rslidar_points", ["/cloud_node"])], [], []]]
    assert load_driver_probe(response)() is True


@pytest.mark.parametrize("publishers", [
    [], [("/rslidar_points", [])], [("/camera/color/image_raw", ["/camera"])],
])
def test_driver_probe_without_lidar_requests_driver_start(publishers):
    response = [1, "system state", [publishers, [], []]]
    assert load_driver_probe(response)() is False


def test_driver_probe_rejects_failed_master_response():
    response = [-1, "master failed", [[("/rslidar_points", ["/cloud_node"])], [], []]]
    assert load_driver_probe(response)() is None
    assert load_driver_probe(error=OSError("master unavailable"))() is None


@pytest.mark.parametrize("driver_present", [False, True])
def test_mapping_start_verifies_standalone_lidar_without_starting_a_second_driver(tmp_path, driver_present):
    tree = ast.parse(STREAM.read_text(encoding="utf-8"))
    function = next(node for node in tree.body
                    if isinstance(node, ast.FunctionDef) and node.name == "mapping_worker")
    calls = []
    replies = []
    recovery_calls = []

    def ensure_lidar():
        recovery_calls.append(True)
        return {"started": not driver_present, "points": 28800, "age_s": 0.05}

    def start_process(arguments, **options):
        calls.append(arguments)
        return SimpleNamespace(pid=1234, poll=lambda: None)

    scope = {
        "mapping_lock": threading.Lock(),
        "mapping_pid": lambda: None,
        "ensure_lidar_runtime": ensure_lidar,
        "MAPPING_LAUNCH": "/deployment/navigation_mapping.launch",
        "MAPPING_LOG": str(tmp_path / "mapping.log"),
        "MAPPING_PID": str(tmp_path / "mapping.pid"),
        "subprocess": SimpleNamespace(Popen=start_process),
        "os": SimpleNamespace(devnull=os.devnull, setsid=lambda: None),
        "time": SimpleNamespace(sleep=lambda seconds: None),
        "emit": lambda kind, data: replies.append((kind, data)),
        "mapping_log_tail": lambda: "",
    }
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(STREAM), "exec"), scope)
    scope["mapping_worker"]("mapping_start")
    assert len(calls) == 1
    assert recovery_calls == [True]
    assert "start_lidar_driver:=true" not in calls[0]
    assert replies[-1][1]["success"] is True


def test_deployment_archive_contains_bridge_dependencies(tmp_path):
    source = ROOT / "lite2_sim/tools/build_lite2_deployment.py"
    spec = importlib.util.spec_from_file_location("mapping_deployment_builder", source)
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    archive = builder.build(tmp_path / "deployment")
    with zipfile.ZipFile(archive) as bundle:
        manifest = json.loads(bundle.read("manifest.json"))
        for relative in ("hardware/execution_guard.py", "hardware/navigation_mapping.launch"):
            data = bundle.read(relative)
            assert data == (ROOT / "lite2_sim" / relative).read_bytes()
            assert manifest["files_sha256"][relative] == hashlib.sha256(data).hexdigest()
