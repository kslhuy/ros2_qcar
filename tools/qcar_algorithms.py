"""Workspace entry point: python tools/qcar_algorithms.py --help."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
    "Development/ros2/src/ros2test/ros2test/multi_vehicle_RealCar"))

from qcar_algorithms.cli import main


if __name__ == "__main__":
    raise SystemExit(main())
