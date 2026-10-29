"""Build an immutable wheel from QCar2_Cran without moving its source files."""

import importlib.util
import json
import os
from pathlib import Path
import shutil

from setuptools import setup
from setuptools.command.build_py import build_py


PROJECT = Path(__file__).resolve().parent
PROVIDER = PROJECT.parent / "Development/ros2/src/ros2test/ros2test/multi_vehicle_RealCar/qcar_algorithms"
spec = importlib.util.spec_from_file_location("qcar_source_build", PROVIDER / "source.py")
source = importlib.util.module_from_spec(spec)
spec.loader.exec_module(source)
ROOT = source.resolve_source(os.environ.get("QCAR_ALGORITHM_BUILD_ROOT", str(PROJECT.parent.parent / "QCar2_Cran")))
MANIFEST = source.describe(ROOT, include_hash=True)
VERSION = "0.1.0+g" + (MANIFEST.get("commit") or "local")[:12] + "." + MANIFEST["sha256"][:12]


class BuildRuntime(build_py):
    def run(self):
        super().run()
        target = Path(self.build_lib) / "qcar_algorithms" / "runtime"
        # setuptools can reuse build/lib across builds. Clear only our verified
        # output directory so removed source files never leak into a new wheel.
        expected_parent = (PROJECT / "build").resolve()
        if not target.resolve().is_relative_to(expected_parent):
            raise RuntimeError(f"Unexpected package output directory: {target}")
        if target.exists():
            shutil.rmtree(target)
        for relative in MANIFEST["files"]:
            destination = target / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / relative, destination)
        if source.snapshot(target) != {k: MANIFEST[k] for k in ("sha256", "files")}:
            raise RuntimeError("Algorithm source changed during packaging; build again.")
        manifest_path = target.parent / "source_manifest.json"
        manifest_path.write_text(json.dumps({**MANIFEST, "version": VERSION}, indent=2), encoding="utf-8")


setup(
    name="qcar-algorithms",
    version=VERSION,
    description="Versioned QCar2_Cran algorithms for simulation and ROS hardware",
    packages=["qcar_algorithms"],
    package_dir={"qcar_algorithms": os.path.relpath(PROVIDER, PROJECT)},
    cmdclass={"build_py": BuildRuntime},
    python_requires=">=3.10",
    install_requires=["numpy", "scipy", "PyYAML", "pyzmq", "matplotlib", "msgpack"],
    extras_require={"neural": ["torch"], "mpc": ["casadi"], "vision": ["opencv-python"]},
    entry_points={"console_scripts": ["qcar-algorithms=qcar_algorithms.cli:main"]},
    zip_safe=False,
)
