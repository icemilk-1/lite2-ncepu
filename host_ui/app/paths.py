"""Resolve bundled resources and persistent workspace data."""
from __future__ import annotations

import os
import sys
from pathlib import Path


def resource_root() -> Path:
    """Return the repository root or the PyInstaller extraction directory."""
    if getattr(sys, "frozen", False) and getattr(sys, "_MEIPASS", None):
        return Path(sys._MEIPASS).resolve()
    return Path(__file__).resolve().parents[2]


def data_root() -> Path:
    """Return the writable workspace root for config, maps, and recordings."""
    override = os.environ.get("LITE2_HOST_UI_DATA_DIR", "").strip()
    if override:
        return Path(override).expanduser().resolve()
    if getattr(sys, "frozen", False):
        local = os.environ.get("LOCALAPPDATA")
        base = Path(local) if local else Path.home() / "AppData" / "Local"
        return (base / "Lite2HostUI").resolve()
    return resource_root()
