#!/usr/bin/env python
"""Preflight checks for the RovLibrary ROS 2 Windows workspace."""

from __future__ import annotations

import importlib
import os
from pathlib import Path
import shutil
import subprocess
import sys


EXPECTED_PYTHON = (3, 8, 3)
EXPECTED_PACKAGES = {
    "rov_msgs",
    "rov_mavlink_bridge",
    "rov_peripherals",
    "rov_control_manager",
    "rov_bringup",
}


class Reporter:
    def __init__(self) -> None:
        self.failures: list[str] = []
        self.warnings: list[str] = []

    def ok(self, label: str, detail: str = "") -> None:
        suffix = f" - {detail}" if detail else ""
        print(f"[OK]   {label}{suffix}")

    def warn(self, label: str, detail: str) -> None:
        self.warnings.append(f"{label}: {detail}")
        print(f"[WARN] {label} - {detail}")

    def fail(self, label: str, detail: str) -> None:
        self.failures.append(f"{label}: {detail}")
        print(f"[FAIL] {label} - {detail}")


def main() -> int:
    repo = Path(__file__).resolve().parents[1]
    reporter = Reporter()

    print("RovLibrary ROS 2 Windows preflight")
    print(f"Workspace: {repo}")
    print()

    check_python(reporter)
    check_import(reporter, "pymavlink")
    check_import(reporter, "rclpy")
    check_ros2(reporter)
    check_colcon(reporter)
    check_workspace_packages(repo, reporter)

    print()
    if reporter.failures:
        print("[RESULT] Preflight failed.")
        for failure in reporter.failures:
            print(f"  - {failure}")
        return 1

    print("[RESULT] Preflight passed.")
    if reporter.warnings:
        print("[RESULT] Warnings:")
        for warning in reporter.warnings:
            print(f"  - {warning}")
    return 0


def check_python(reporter: Reporter) -> None:
    current = sys.version_info[:3]
    current_text = ".".join(str(part) for part in current)
    expected_text = ".".join(str(part) for part in EXPECTED_PYTHON)
    if current == EXPECTED_PYTHON:
        reporter.ok("Python version", current_text)
        return
    reporter.fail("Python version", f"{current_text}, expected {expected_text}")


def check_import(reporter: Reporter, module_name: str) -> None:
    try:
        module = importlib.import_module(module_name)
    except Exception as exc:  # noqa: BLE001
        reporter.fail(f"import {module_name}", str(exc))
        return

    version = getattr(module, "__version__", None)
    detail = f"version {version}" if version else "imported"
    reporter.ok(f"import {module_name}", detail)


def check_ros2(reporter: Reporter) -> None:
    ros2_path = shutil.which("ros2")
    if ros2_path is None:
        reporter.fail("ros2 command", "not found on PATH")
        return

    reporter.ok("ros2 command", ros2_path)
    distro = os.environ.get("ROS_DISTRO", "")
    if distro.lower() == "humble":
        reporter.ok("ROS_DISTRO", distro)
    elif distro:
        reporter.fail("ROS_DISTRO", f"{distro}, expected humble")
    else:
        reporter.warn("ROS_DISTRO", "not set; did you source ROS 2 Humble setup.bat?")

    result = run_command(["ros2", "--help"])
    if result.returncode == 0:
        reporter.ok("ros2 --help")
    else:
        reporter.fail("ros2 --help", result.stderr_or_stdout())


def check_colcon(reporter: Reporter) -> None:
    colcon_path = shutil.which("colcon")
    if colcon_path is not None:
        reporter.ok("colcon command", colcon_path)
        return

    result = run_command([sys.executable, "-m", "colcon", "--help"])
    if result.returncode == 0:
        reporter.ok("python -m colcon")
    else:
        reporter.fail("colcon command", "not found on PATH and python -m colcon failed")


def check_workspace_packages(repo: Path, reporter: Reporter) -> None:
    src = repo / "src"
    if not src.exists():
        reporter.fail("workspace src", f"missing: {src}")
        return

    discovered = discover_with_colcon(src)
    method = "colcon list"
    if discovered is None:
        discovered = discover_package_xml(src)
        method = "package.xml scan"

    missing = sorted(EXPECTED_PACKAGES - discovered)
    extra = sorted(discovered - EXPECTED_PACKAGES)
    if missing:
        reporter.fail("workspace package discovery", f"missing {missing} via {method}")
        return

    detail = f"{', '.join(sorted(discovered))} via {method}"
    if extra:
        detail += f"; extra packages: {', '.join(extra)}"
    reporter.ok("workspace package discovery", detail)


def discover_with_colcon(src: Path) -> set[str] | None:
    result = run_command(["colcon", "list", "--base-paths", str(src), "--names-only"])
    if result.returncode != 0:
        result = run_command(
            [sys.executable, "-m", "colcon", "list", "--base-paths", str(src), "--names-only"]
        )
    if result.returncode != 0:
        return None
    return {line.strip() for line in result.stdout.splitlines() if line.strip()}


def discover_package_xml(src: Path) -> set[str]:
    packages = set()
    for package_xml in src.glob("*/package.xml"):
        text = package_xml.read_text(encoding="utf-8", errors="replace")
        start = text.find("<name>")
        end = text.find("</name>")
        if start != -1 and end != -1 and end > start:
            packages.add(text[start + len("<name>") : end].strip())
    return packages


class CommandResult:
    def __init__(self, completed: subprocess.CompletedProcess[str] | None, exc: Exception | None = None) -> None:
        self.completed = completed
        self.exc = exc
        self.returncode = completed.returncode if completed is not None else 1
        self.stdout = completed.stdout if completed is not None else ""
        self.stderr = completed.stderr if completed is not None else ""

    def stderr_or_stdout(self) -> str:
        if self.exc is not None:
            return str(self.exc)
        text = (self.stderr or self.stdout or "").strip()
        return text if text else f"exit code {self.returncode}"


def run_command(args: list[str]) -> CommandResult:
    try:
        completed = subprocess.run(
            args,
            check=False,
            capture_output=True,
            text=True,
            timeout=30,
        )
        return CommandResult(completed)
    except Exception as exc:  # noqa: BLE001
        return CommandResult(None, exc)


if __name__ == "__main__":
    raise SystemExit(main())

