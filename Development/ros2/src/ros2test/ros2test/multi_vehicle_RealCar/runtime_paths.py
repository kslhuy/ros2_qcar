from pathlib import Path
from typing import Optional


_PACKAGE_NAME = "ros2test"


def _current_package_root(current_file: str) -> Path:
    current_path = Path(current_file).resolve()
    for parent in [current_path.parent, *current_path.parents]:
        if (parent / "multi_vehicle_RealCar").is_dir() and (parent / "qcar").is_dir():
            return parent
    return current_path.parent


def _source_package_root_from_colcon_layout(current_file: str) -> Optional[Path]:
    current_path = Path(current_file).resolve()
    for parent in current_path.parents:
        if parent.name not in {"install", "build"}:
            continue
        workspace_root = parent.parent
        candidate = workspace_root / "src" / _PACKAGE_NAME / _PACKAGE_NAME
        if (candidate / "multi_vehicle_RealCar").is_dir() and (candidate / "qcar").is_dir():
            return candidate
    return None


def get_preferred_package_root(current_file: str) -> Path:
    source_root = _source_package_root_from_colcon_layout(current_file)
    if source_root is not None:
        return source_root
    return _current_package_root(current_file)


def get_preferred_qcar_dir(current_file: str) -> Path:
    return get_preferred_package_root(current_file) / "qcar"


def get_preferred_trust_log_dir(current_file: str) -> Path:
    return (
        get_preferred_package_root(current_file)
        / "multi_vehicle_RealCar"
        / "Observer"
        / "TrustbasedDistributedObserver"
    )
