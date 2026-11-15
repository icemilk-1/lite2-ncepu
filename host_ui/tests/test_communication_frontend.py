import shutil
import subprocess
from pathlib import Path

import pytest


def test_frontend_connection_watchdog_and_backpressure():
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node.js is required for frontend regression checks")
    script = Path(__file__).with_name("communication_frontend.cjs")
    result = subprocess.run([node, str(script)], capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
