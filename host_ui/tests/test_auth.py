"""鉴权：Origin 白名单、访问令牌、非回环绑定强制令牌。"""
from __future__ import annotations

import pytest
from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect

from host_ui.app.config import Config
from host_ui.app.server import create_app


def make_app(**web) -> TestClient:
    config = Config({"web": {"host": "127.0.0.1", "port": 8710, **web},
                     "link": {"kind": "mock"}})
    return TestClient(create_app(config, auto_connect=False))


def test_rest_requires_token_when_configured() -> None:
    client = make_app(token="secret-token")
    with client:
        assert client.get("/api/status").status_code == 403
        assert client.get("/api/status",
                          headers={"x-auth-token": "wrong"}).status_code == 403
        assert client.get("/api/status",
                          headers={"x-auth-token": "secret-token"}).status_code == 200


def test_rest_allowed_from_loopback_without_token() -> None:
    client = make_app()
    with client:
        assert client.get("/api/status").status_code == 200


def test_ws_rejects_bad_origin() -> None:
    client = make_app()
    with client:
        with pytest.raises(WebSocketDisconnect) as error:
            with client.websocket_connect(
                    "/ws", headers={"origin": "http://evil.example"}) as ws:
                ws.receive_json()
        assert error.value.code == 1008


def test_ws_accepts_allowed_origin() -> None:
    client = make_app()
    with client:
        with client.websocket_connect(
                "/ws", headers={"origin": "http://127.0.0.1:8710"}) as ws:
            hello = ws.receive_json()
            assert hello["type"] == "hello"


def test_ws_requires_token() -> None:
    client = make_app(token="secret-token")
    with client:
        with pytest.raises(WebSocketDisconnect):
            with client.websocket_connect("/ws") as ws:
                ws.receive_json()
        with client.websocket_connect("/ws?token=secret-token",
                                      headers={"origin": "http://127.0.0.1:8710"}) as ws:
            assert ws.receive_json()["type"] == "hello"


def test_non_loopback_requires_token() -> None:
    config = Config({"web": {"host": "0.0.0.0", "port": 8710, "token": ""},
                     "link": {"kind": "mock"}})
    with pytest.raises(RuntimeError):
        create_app(config, auto_connect=False)