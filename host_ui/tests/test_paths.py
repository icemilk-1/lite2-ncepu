from pathlib import Path
import sys

from host_ui.app.paths import data_root, resource_root


def test_source_paths_keep_repository_root(monkeypatch):
    monkeypatch.delenv("LITE2_HOST_UI_DATA_DIR", raising=False)
    monkeypatch.setattr(sys, "frozen", False, raising=False)
    expected = Path(__file__).resolve().parents[2]
    assert resource_root() == expected
    assert data_root() == expected


def test_frozen_resources_and_data_have_separate_roots(monkeypatch, tmp_path):
    bundle = tmp_path / "bundle"
    local = tmp_path / "local"
    monkeypatch.setattr(sys, "frozen", True, raising=False)
    monkeypatch.setattr(sys, "_MEIPASS", str(bundle), raising=False)
    monkeypatch.setenv("LOCALAPPDATA", str(local))
    monkeypatch.delenv("LITE2_HOST_UI_DATA_DIR", raising=False)
    assert resource_root() == bundle.resolve()
    assert data_root() == (local / "Lite2HostUI").resolve()


def test_explicit_data_root_preserves_bundled_resource_path(monkeypatch, tmp_path):
    bundle, data = tmp_path / "bundle", tmp_path / "data"
    monkeypatch.setattr(sys, "frozen", True, raising=False)
    monkeypatch.setattr(sys, "_MEIPASS", str(bundle), raising=False)
    monkeypatch.setenv("LITE2_HOST_UI_DATA_DIR", str(data))
    assert resource_root() == bundle.resolve()
    assert data_root() == data.resolve()
