"""Collect platform and ROS 1 interface information using read-only commands."""

import argparse
import json
import platform
from pathlib import Path
import subprocess
import sys


def inspect_command(command):
    try:
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                universal_newlines=True, timeout=10)
        return {"command": command, "returncode": result.returncode,
                "stdout": result.stdout.strip(), "stderr": result.stderr.strip()}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"command": command, "returncode": None, "error": str(error)}


def collect(topics):
    commands = [["rosversion", "-d"], ["rostopic", "list"], ["rosnode", "list"]]
    commands.extend(["rostopic", "info", topic] for topic in topics)
    checks = [inspect_command(command) for command in commands]
    return {"platform": platform.platform(), "architecture": platform.machine(),
            "python": sys.version, "checks": checks,
            "ros_inspection_passed": all(check["returncode"] == 0 for check in checks)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", action="append", default=[], help="Inspect one actual topic; repeat as needed.")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = collect(args.topic)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"ros_inspection_passed": report["ros_inspection_passed"],
                      "report": str(args.output)}, ensure_ascii=False))
    return 0 if report["ros_inspection_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
