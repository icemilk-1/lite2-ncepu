"""host_ui 测试公共配置。

1. 保证无论从哪个目录调用 pytest，都能 ``import host_ui``。
2. 覆盖 pytest 的 ``tmp_path``：本机平台临时目录（DSH 注入的 TEMP）里
   已有目录 ACL 受限，pytest 自带的 ``pytest-of-<user>`` 根目录会抛
   ``PermissionError``；这里改用仓库内可控目录，测试仍然完全离线。
"""
import shutil
import sys
import uuid
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

_TMP_ROOT = Path(__file__).resolve().parent / ".pytest-tmp"


@pytest.fixture
def tmp_path():
    """函数级临时目录（替代 pytest 内置实现，见模块 docstring）。"""
    _TMP_ROOT.mkdir(parents=True, exist_ok=True)
    path = _TMP_ROOT / uuid.uuid4().hex
    path.mkdir()
    try:
        yield path
    finally:
        shutil.rmtree(path, ignore_errors=True)


def pytest_sessionfinish(session, exitstatus):  # noqa: ARG001 - pytest 钩子签名
    """测试结束后清理空的临时根目录（并发运行时会失败，忽略即可）。"""
    try:
        _TMP_ROOT.rmdir()
    except OSError:
        pass