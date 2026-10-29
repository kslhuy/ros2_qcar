"""Source selection without importing algorithms, ROS, or hardware drivers.

The existing algorithms use flat imports (``Observer``, ``vehicle_logic``, etc.).
Select the whole runtime together so its factories and support modules agree.
"""

import hashlib
import importlib.abc
import importlib.machinery
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys


SOURCE_ENV = "QCAR_ALGORITHM_SOURCE"
CRAN_RUNTIME = Path("Development/multi_vehicle_self_driving_RealQcar/qcar")
ROS_RUNTIME = Path("Development/ros2/src/ros2test/ros2test/multi_vehicle_RealCar")
REQUIRED_FILES = (
    "vehicle_logic.py", "config_main.py", "command_types.py",
    "Observer/local_state_estimators.py", "Observer/fleet_state_estimators.py",
    "Controller/controller_manager.py", "StateMachine/vehicle_state_machine.py",
    "V2V/v2v_manager.py", "PathPlanner/path_rich.py",
    "Calibration/calibration_utils.py", "Taxi/taxi_manager.py", "Yolo/YoLo.py",
)
EXCLUDED_DIRS = {
    ".git", ".serena", ".pytest_cache", "__pycache__", "node_modules",
    "build", "dist", "logs", "data_logs", "scope_recordings",
    "observer_recordings", "test_recordings", "neural_obs_recordings",
    "results", "datasets", "Rapport_paper", "Origine", "fig", "notebook", "notebooks",
    "GUI", "QcarAdaptRos", "limo", "qcar_algorithms",
}
TEXT_SUFFIXES = {".py", ".yaml", ".yml", ".json", ".txt", ".cpp", ".c", ".h", ".hpp"}
MODEL_SUFFIXES = {".pt", ".pth", ".onnx", ".npz", ".npy", ".mat"}


def resolve_source(value, embedded_root=None):
    """Accept a repo, a qcar runtime directory, 'embedded', or 'package'."""
    if not value or value == "embedded":
        if embedded_root is None:
            embedded_root = Path(__file__).resolve().parent.parent
        root = Path(embedded_root).resolve()
    elif value == "package":
        try:
            distribution = importlib.metadata.distribution("qcar-algorithms")
        except importlib.metadata.PackageNotFoundError as exc:
            raise ValueError("Install a qcar-algorithms wheel before selecting 'package'.") from exc
        root = Path(distribution.locate_file("qcar_algorithms/runtime")).resolve()
    else:
        root = Path(value).expanduser().resolve()
        for relative in (CRAN_RUNTIME, ROS_RUNTIME):
            if (root / relative / "vehicle_logic.py").is_file():
                root = root / relative
                break
    missing = [name for name in REQUIRED_FILES if not (root / name).is_file()]
    if missing:
        raise ValueError(f"Incomplete algorithm source {root}: missing {', '.join(missing)}")
    return root


def runtime_files(root):
    """Runtime code, configuration, weights, and calibration tables for export.

    Logs, training datasets, manuscripts, caches, and native build products stay
    in the development project. Native C/C++ source is included for rebuilding.
    """
    root = Path(root)
    for directory, dirs, names in os.walk(root):
        dirs[:] = sorted(d for d in dirs if d not in EXCLUDED_DIRS and not d.startswith("."))
        for name in sorted(names):
            file = Path(directory) / name
            relative = file.relative_to(root)
            if file.is_symlink():
                continue
            if name.startswith("vehicle_main") or name.endswith(".train_history.json"):
                continue
            suffix = file.suffix.lower()
            table = suffix == ".csv" and any(
                part.lower() in {"models", "cfg", "steering_lookup"}
                for part in relative.parts[:-1]
            )
            if suffix in TEXT_SUFFIXES or suffix in MODEL_SUFFIXES or table:
                yield relative


def fingerprint(files):
    digest = hashlib.sha256()
    for name, file_hash in sorted(files.items()):
        digest.update(f"{name}\0{file_hash}\n".encode("utf-8"))
    return digest.hexdigest()


def snapshot(root):
    root = Path(root)
    files = {path.as_posix(): hashlib.sha256((root / path).read_bytes()).hexdigest()
             for path in runtime_files(root)}
    return {"sha256": fingerprint(files), "files": files}


def describe(root, include_hash=False):
    root = Path(root).resolve()
    manifest = root.parent / "source_manifest.json"
    if manifest.is_file():
        info = json.loads(manifest.read_text(encoding="utf-8"))
        return {**info, "root": str(root), "mode": "package"}
    info = {"root": str(root), "mode": "source", "commit": None, "dirty": None}
    try:
        repo = next(parent for parent in (root, *root.parents) if (parent / ".git").exists())
        command = ["git", "-c", f"safe.directory={repo.as_posix()}", "-C", str(repo)]
        info["commit"] = subprocess.run(command + ["rev-parse", "HEAD"],
            capture_output=True, text=True, timeout=10, check=True).stdout.strip()
        changed = subprocess.run(command + ["status", "--porcelain", "--untracked-files=normal",
            "--ignore-submodules=all", "--", root.relative_to(repo).as_posix()],
            capture_output=True, text=True, timeout=10, check=True)
        info["dirty"] = bool(changed.stdout.strip())
    except (OSError, ValueError, StopIteration, subprocess.SubprocessError):
        pass
    if include_hash:
        info.update(snapshot(root))
    return info


def verify_package(root):
    """Reject a modified deployment snapshot rather than loading mixed code."""
    manifest = Path(root).parent / "source_manifest.json"
    if not manifest.is_file():
        raise ValueError("The installed algorithm package has no source manifest.")
    expected = json.loads(manifest.read_text(encoding="utf-8"))
    actual = snapshot(root)
    if actual["sha256"] != expected.get("sha256") or actual["files"] != expected.get("files"):
        raise ValueError("Installed algorithm files differ from the package manifest; reinstall the wheel.")
    return expected


def managed_names(*roots):
    names = set()
    for root in roots:
        for path in Path(root).iterdir():
            if path.name in EXCLUDED_DIRS or path.name.startswith("."):
                continue
            if path.is_file() and path.suffix == ".py":
                if not path.stem.startswith(("vehicle_main", "test_")) and path.stem != "__init__":
                    names.add(path.stem)
            elif path.is_dir() and any(path.rglob("*.py")):
                names.add(path.name)
    return names


class _SourceFinder(importlib.abc.MetaPathFinder):
    def __init__(self, root, names):
        self.root = Path(root)
        self.names = names

    def find_spec(self, fullname, path=None, target=None):
        if path is not None or fullname not in self.names:
            return None
        spec = importlib.machinery.PathFinder.find_spec(fullname, [str(self.root)])
        if spec is None:
            raise ModuleNotFoundError(
                f"{fullname!r} is absent from selected algorithm source {self.root}; "
                "refusing to combine it with the embedded runtime.", name=fullname,
            )
        return spec


def activate(embedded_root, selection=None):
    """Select a source before any flat algorithm imports have happened.

    No selection preserves the deployed ROS runtime. An explicitly selected
    invalid source fails immediately; it never silently reverts to local code.
    """
    selection = os.environ.get(SOURCE_ENV, "embedded") if selection is None else selection
    embedded_root = Path(embedded_root).resolve()
    root = resolve_source(selection, embedded_root)
    if root == embedded_root:
        if any(isinstance(finder, _SourceFinder) for finder in sys.meta_path):
            raise RuntimeError("Restart Python before switching back to embedded algorithms.")
        return {"root": str(root), "mode": "embedded"}
    if selection == "package":
        info = {**verify_package(root), "root": str(root), "mode": "package"}
    else:
        info = describe(root)
    names = managed_names(embedded_root, root)
    for name, module in tuple(sys.modules.items()):
        if name.split(".", 1)[0] not in names:
            continue
        origin = getattr(module, "__file__", None)
        locations = [origin] if origin else list(getattr(module, "__path__", []))
        if any(not Path(location).resolve().is_relative_to(root) for location in locations):
            raise RuntimeError(f"{name} was already imported from {origin}; select algorithms before imports.")
    for finder in sys.meta_path:
        if isinstance(finder, _SourceFinder):
            if finder.root != root:
                raise RuntimeError("Restart Python before switching the algorithm source.")
            return info
    sys.path.insert(0, str(root))
    sys.meta_path.insert(0, _SourceFinder(root, names))
    importlib.invalidate_caches()
    return info
