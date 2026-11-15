"""会话录制：把遥测与事件写成可回溯的 JSONL 文件。

目录结构（默认 ``host_ui/sessions``）::

    sessions/
      20261114_113015_ab12cd/
        meta.json        # 开始时间、链路、限幅、版本
        telemetry.jsonl  # 10 Hz 位姿/指令/安全状态
        events.jsonl     # 连接、急停、模式变化、碰撞等事件

JSONL 便于直接 grep，也能被 pandas / 回放工具逐行读取。
"""
from __future__ import annotations

import json
import threading
import time
import uuid
import zipfile
from datetime import datetime
from pathlib import Path
from typing import Any

from .paths import data_root


def _repo_root() -> Path:
    return data_root()


class Recorder:
    """线程安全的会话录制器。"""

    def __init__(self, root: str | Path | None = None) -> None:
        if root is None:
            root = _repo_root() / "host_ui" / "sessions"
        self.root = Path(root)
        self._lock = threading.RLock()
        self._dir: Path | None = None
        self._telemetry = None
        self._events = None
        self._rows = 0
        self._events_count = 0
        self._started_at = 0.0
        self._meta: dict[str, Any] = {}

    # ------------------------------------------------------------------ 生命周期
    def start(self, metadata: dict | None = None, prefix: str = "session") -> Path:
        with self._lock:
            if self._dir is not None:
                return self._dir
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            short = uuid.uuid4().hex[:6]
            session_dir = self.root / f"{prefix}_{stamp}_{short}"
            session_dir.mkdir(parents=True, exist_ok=True)
            meta = {
                "started_at": time.time(),
                "started_at_text": datetime.now().isoformat(timespec="seconds"),
                "python_version": __import__("sys").version,
                **(metadata or {}),
            }
            (session_dir / "meta.json").write_text(
                json.dumps(meta, ensure_ascii=False, indent=2, default=str),
                encoding="utf-8")
            self._dir = session_dir
            self._telemetry = (session_dir / "telemetry.jsonl").open(
                "a", encoding="utf-8")
            self._events = (session_dir / "events.jsonl").open(
                "a", encoding="utf-8")
            self._rows = 0
            self._events_count = 0
            self._started_at = time.monotonic()
            self._meta = meta
            return session_dir

    def stop(self) -> Path | None:
        with self._lock:
            session_dir = self._dir
            for handle in (self._telemetry, self._events):
                if handle is not None:
                    try:
                        handle.flush()
                        handle.close()
                    except Exception:
                        pass
            self._telemetry = None
            self._events = None
            self._dir = None
            if session_dir is not None:
                meta_path = session_dir / "meta.json"
                try:
                    meta = json.loads(meta_path.read_text(encoding="utf-8"))
                    meta["stopped_at"] = time.time()
                    meta["stopped_at_text"] = datetime.now().isoformat(timespec="seconds")
                    meta["rows"] = self._rows
                    meta["events"] = self._events_count
                    meta_path.write_text(
                        json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
                except Exception:
                    pass
            return session_dir

    # ------------------------------------------------------------------ 写入
    def write_row(self, row: dict) -> None:
        handle = self._telemetry
        if handle is None:
            return
        with self._lock:
            try:
                handle.write(json.dumps(row, ensure_ascii=False,
                                        separators=(",", ":")) + "\n")
                self._rows += 1
                if self._rows % 50 == 0:
                    handle.flush()
            except Exception:
                pass

    def write_event(self, event: dict) -> None:
        handle = self._events
        if handle is None:
            return
        with self._lock:
            try:
                handle.write(json.dumps(event, ensure_ascii=False,
                                        separators=(",", ":")) + "\n")
                self._events_count += 1
                handle.flush()
            except Exception:
                pass

    # ------------------------------------------------------------------ 状态
    @property
    def active(self) -> bool:
        return self._dir is not None

    @property
    def directory(self) -> Path | None:
        return self._dir

    def status(self) -> dict:
        with self._lock:
            return {
                "active": self._dir is not None,
                "path": str(self._dir) if self._dir else "",
                "rows": self._rows,
                "events": self._events_count,
                "elapsed_s": round(time.monotonic() - self._started_at, 1)
                if self._dir else 0.0,
            }

    def latest_directory(self) -> Path | None:
        """最近一次会话目录（用于下载）。"""
        if self._dir is not None:
            return self._dir
        if not self.root.exists():
            return None
        candidates = sorted(
            (path for path in self.root.iterdir() if path.is_dir()),
            key=lambda path: path.stat().st_mtime, reverse=True)
        return candidates[0] if candidates else None

    def archive_latest(self) -> Path | None:
        """把最近一次会话打包成 zip（放在 sessions 目录下）。"""
        session = self.latest_directory()
        if session is None:
            return None
        target = self.root / f"{session.name}.zip"
        with zipfile.ZipFile(target, "w", zipfile.ZIP_DEFLATED) as archive:
            for path in sorted(session.rglob("*")):
                if path.is_file():
                    archive.write(path, arcname=f"{session.name}/{path.name}")
        return target
