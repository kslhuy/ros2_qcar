"""Inspect, launch, and package the canonical algorithm source."""

import argparse
import ast
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

from .source import SOURCE_ENV, describe, resolve_source, snapshot


def embedded_root():
    return Path(__file__).resolve().parent.parent


def workspace_root():
    return next((p for p in Path(__file__).resolve().parents if (p / "shared_algorithms").is_dir()), None)


def default_source():
    if os.environ.get(SOURCE_ENV):
        return os.environ[SOURCE_ENV]
    workspace = workspace_root()
    if workspace and (workspace.parent / "QCar2_Cran").is_dir():
        return str(workspace.parent / "QCar2_Cran")
    return "embedded" if (embedded_root() / "vehicle_logic.py").is_file() else "package"


def compatibility(root):
    """Check the ROS wrapper's called methods without importing hardware code."""
    wrapper = embedded_root() / "vehicle_main_ros_qcar.py"
    if not wrapper.is_file():
        return []
    required = set()
    for node in ast.walk(ast.parse(wrapper.read_text(encoding="utf-8-sig"))):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
            continue
        owner = node.func.value
        if (isinstance(owner, ast.Attribute) and owner.attr == "vehicle_logic" and
                isinstance(owner.value, ast.Name) and owner.value.id == "self"):
            required.add(node.func.attr)
    tree = ast.parse((root / "vehicle_logic.py").read_text(encoding="utf-8-sig"))
    logic = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "VehicleLogic")
    provided = {n.name for n in logic.body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))}
    return sorted(required - provided)


def check(root):
    missing = compatibility(root)
    if missing:
        raise ValueError("VehicleLogic methods required by ROS are missing: " + ", ".join(missing))
    info = describe(root, include_hash=True)
    summary = {k: v for k, v in info.items() if k != "files"}
    summary["runtime_files"] = len(info["files"])
    summary["ros_method_check"] = (
        "passed (static; no hardware was started)"
        if (embedded_root() / "vehicle_main_ros_qcar.py").is_file()
        else "not available; run check from the ROS checkout for the wrapper method check"
    )
    print(json.dumps(summary, indent=2), flush=True)
    return info


def compare(root):
    if not (embedded_root() / "vehicle_logic.py").is_file():
        raise ValueError("Compare from the ROS checkout, which contains the embedded baseline.")
    local = snapshot(embedded_root())["files"]
    source = snapshot(root)["files"]
    groups = {}
    for name in sorted(local.keys() | source.keys()):
        group = name.split("/", 1)[0] if "/" in name else "runtime helpers"
        row = groups.setdefault(group, {"identical": 0, "different": 0, "embedded_only": 0, "source_only": 0})
        key = ("source_only" if name not in local else "embedded_only" if name not in source
               else "identical" if local[name] == source[name] else "different")
        row[key] += 1
    print(json.dumps(groups, indent=2))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="action", required=True)
    for name in ("check", "compare", "run", "package"):
        command = commands.add_parser(name)
        command.add_argument("--source", default=default_source(),
                             help="QCar2_Cran repo, qcar directory, embedded, or package")
        if name == "run":
            command.add_argument("command", nargs=argparse.REMAINDER,
                                 help="Command after --, e.g. ros2 run ros2test vehicle_main_ros_qcar")
        if name == "package":
            command.add_argument("--output", default="dist/algorithms")
    args = parser.parse_args(argv)
    try:
        root = resolve_source(args.source, embedded_root())
        if args.action == "compare":
            compare(root)
        elif args.action == "package":
            workspace = workspace_root()
            if workspace is None:
                raise ValueError("Build packages from the ros2_qcar checkout, which contains shared_algorithms/.")
            if importlib.util.find_spec("wheel") is None:
                raise ValueError("The selected Python environment needs the wheel package to build a deployment wheel.")
            check(root)
            env = dict(os.environ, QCAR_ALGORITHM_BUILD_ROOT=str(root))
            return subprocess.call([sys.executable, "-m", "pip", "wheel", "--no-index", "--no-deps", "--no-build-isolation",
                "--wheel-dir", str(Path(args.output).resolve()), str(workspace / "shared_algorithms")], env=env)
        elif args.action == "run":
            command = args.command[1:] if args.command[:1] == ["--"] else args.command
            if not command:
                raise ValueError("Supply the command to run after --.")
            check(root)
            # Preserve 'package' so activation verifies the installed manifest.
            selection = "package" if args.source == "package" else str(root)
            env = dict(os.environ, **{SOURCE_ENV: selection})
            return subprocess.call(command, env=env)
        else:
            if args.source == "package":
                from .source import verify_package
                verify_package(root)
            check(root)
    except (ValueError, RuntimeError, OSError, SyntaxError) as exc:
        print(f"Algorithm source error: {exc}", file=sys.stderr)
        return 1
    return 0
