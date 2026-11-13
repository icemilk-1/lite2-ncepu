"""Export the portable navigation core, offline tests and hardware preparation files."""

import argparse
import ast
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import zipfile


PROJECT = Path(__file__).resolve().parents[1]
CORE = Path("controllers/sensor_capture/navigation_core")
MODULES = ("__init__.py", "contracts.py", "control.py", "grid.py", "planner.py",
           "session.py", "terrain.py", "course.py", "evaluation.py",
           "execution_adapters/__init__.py", "execution_adapters/ros1_twist.py")


def build(output):
    files = [CORE / module for module in MODULES]
    files.extend(Path(name) for name in (
        "tools/replay_navigation.py", "tools/collect_lite2_environment.py",
        "tests/test_navigation_core.py", "hardware/README.md", "hardware/profile.example.json"))
    hashes = {}
    for relative in files:
        data = (PROJECT / relative).read_bytes()
        if relative.suffix == ".py":
            ast.parse(data.decode("utf-8"), filename=str(relative), feature_version=(3, 6))
        hashes[relative.as_posix()] = hashlib.sha256(data).hexdigest()
    manifest = {"created_at_utc": datetime.now(timezone.utc).isoformat(),
                "python_syntax_minimum": "3.6", "runtime_dependency": "numpy>=1.19.5",
                "files_sha256": hashes, "hardware_interface_status": "awaiting_actual_sensor_and_control_configuration"}
    output.mkdir(parents=True, exist_ok=True)
    archive = output / "lite2_navigation_core.zip"
    with zipfile.ZipFile(archive, "x", compression=zipfile.ZIP_DEFLATED) as bundle:
        for relative in files:
            bundle.write(PROJECT / relative, relative.as_posix())
        bundle.writestr("manifest.json", json.dumps(manifest, indent=2))
        bundle.writestr("requirements.txt", "numpy>=1.19.5\n")
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return archive


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=PROJECT / "runs" /
                        ("lite2_deployment_" + datetime.now().strftime("%Y%m%d_%H%M%S")))
    args = parser.parse_args()
    print(build(args.output))


if __name__ == "__main__":
    main()
