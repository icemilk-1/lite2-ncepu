"""Real map freshness uses source age, independent of state/SSH liveness."""
import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

import host_ui.app.transports.ssh_ros as ssh_module
from host_ui.app.transports.base import TransportCallbacks
from host_ui.app.transports.ssh_ros import SshRosTransport


def setup_transport(monkeypatch, **options):
    clock = [100.0]
    monkeypatch.setattr(ssh_module, "time", SimpleNamespace(monotonic=lambda: clock[0]))
    maps = []
    transport = SshRosTransport(options, TransportCallbacks(
        on_frame=lambda frame: None, on_event=lambda *args: None, on_map=maps.append))
    transport.last_frame = clock[0]
    return transport, clock, maps


def receive_map(transport, **metadata):
    payload = {"width": 2, "height": 2, "resolution": 0.05,
               "origin": [0, 0, 0], "data": [0, 0, -1, 100], "name": "map"}
    payload.update(metadata)
    transport._receive({"type": "map", "data": payload})


def test_map_expires_while_robot_state_stays_fresh(monkeypatch):
    transport, clock, maps = setup_transport(monkeypatch)
    receive_map(transport, stamp=1234.0, age_s=0.25)
    health = transport.status()["map_health"]
    assert health["valid"]
    assert health["age_s"] == pytest.approx(0.25)
    assert health["source_stamp"] == 1234.0
    assert len(maps) == 1
    clock[0] += 8.0
    transport.last_frame = clock[0]  # state and /odom can live while octomap stalls
    status = transport.status()
    assert status["connected"]
    assert not status["map_health"]["valid"]
    assert status["map_health"]["age_s"] == pytest.approx(8.25)
    assert "过期" in status["map_health"]["reason"]


def test_old_map_republication_cannot_refresh_source_age(monkeypatch):
    transport, clock, maps = setup_transport(monkeypatch)
    receive_map(transport, stamp=1234.0, age_s=1.0)
    clock[0] += 7.5
    transport.last_frame = clock[0]
    # Even if the sender incorrectly resets its reported age, same stamp stays old.
    receive_map(transport, stamp=1234.0, age_s=0.0)
    health = transport.status()["map_health"]
    assert not health["valid"]
    assert health["age_s"] == pytest.approx(8.5)
    assert health["received_age_s"] == 0.0
    receive_map(transport, stamp=1242.0, age_s=0.1)
    assert transport.status()["map_health"]["valid"]


@pytest.mark.parametrize("metadata", [
    {}, {"stamp": 0.0, "age_s": 0.0}, {"stamp": 1, "age_s": -1},
    {"stamp": 1, "age_s": float("nan")}, {"stamp": float("inf"), "age_s": 0},
    {"stamp": 1, "age_s": 30.0},
])
def test_missing_invalid_or_stale_source_time_cannot_enable_map(monkeypatch, metadata):
    transport, clock, maps = setup_transport(monkeypatch)
    receive_map(transport, **metadata)
    assert len(maps) == 1  # operator may inspect the map, autonomous use is forbidden
    health = transport.status()["map_health"]
    assert not health["valid"]
    assert health["reason"]


def test_regressed_map_does_not_replace_current_grid(monkeypatch):
    transport, clock, maps = setup_transport(monkeypatch)
    receive_map(transport, stamp=20, age_s=0)
    receive_map(transport, stamp=19, age_s=0)
    assert len(maps) == 1
    assert not transport.status()["map_health"]["valid"]
    receive_map(transport, stamp=21, age_s=0)
    assert transport.status()["map_health"]["valid"]


def test_invalid_time_between_republications_cannot_erase_source_age(monkeypatch):
    transport, clock, maps = setup_transport(monkeypatch)
    receive_map(transport, stamp=20, age_s=0)
    clock[0] += 9
    transport.last_frame = clock[0]
    receive_map(transport, stamp=20)  # missing source age must preserve the old baseline
    receive_map(transport, stamp=20, age_s=0)
    health = transport.status()["map_health"]
    assert not health["valid"]
    assert health["age_s"] == pytest.approx(9)


def test_connection_reset_requires_a_new_map_even_after_state_resumes(monkeypatch):
    transport, clock, maps = setup_transport(monkeypatch)
    receive_map(transport, stamp=20, age_s=0)
    transport._close_connection()
    transport.last_frame = clock[0]
    health = transport.status()["map_health"]
    assert not health["valid"]
    assert health["age_s"] is None
    receive_map(transport, stamp=21, age_s=0)
    assert transport.status()["map_health"]["valid"]


def test_map_timeout_is_configurable_and_bounded(monkeypatch):
    transport, _, _ = setup_transport(monkeypatch, map_timeout_s=6)
    assert transport.map_timeout_s == 6
    assert SshRosTransport.default_options["map_timeout_s"] == 8
    assert SshRosTransport({"map_timeout_s": 100}, transport.callbacks).map_timeout_s == 30
    assert SshRosTransport({"map_timeout_s": -1}, transport.callbacks).map_timeout_s == 1


def test_onboard_stream_emits_source_time_without_cross_host_clock_comparison():
    path = Path(__file__).resolve().parents[1] / "app/transports/ssh_ros_stream.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    function = next(node for node in tree.body
                    if isinstance(node, ast.FunctionDef) and node.name == "map_received")
    emitted = []
    scope = {"math": __import__("math"),
             "rospy": SimpleNamespace(Time=SimpleNamespace(
                 now=lambda: SimpleNamespace(to_sec=lambda: 101.5))),
             "emit": lambda kind, data: emitted.append((kind, data))}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), "exec"), scope)
    origin = SimpleNamespace(position=SimpleNamespace(x=0, y=0),
                             orientation=SimpleNamespace(x=0, y=0, z=0, w=1))
    message = SimpleNamespace(info=SimpleNamespace(origin=origin, width=2, height=2,
                                                   resolution=0.05),
                              header=SimpleNamespace(frame_id="map", stamp=SimpleNamespace(
                                  to_sec=lambda: 100.0)), data=[0, 0, -1, 100])
    scope["map_received"](message)
    assert emitted[0][1]["stamp"] == 100.0
    assert emitted[0][1]["age_s"] == 1.5
