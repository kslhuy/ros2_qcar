"""Source isolation and deployment integrity; these tests never start a car."""

import importlib
import json
from pathlib import Path
import sys
import types

import pytest

RUNTIME = Path(__file__).resolve().parents[1] / "Development/ros2/src/ros2test/ros2test/multi_vehicle_RealCar"
sys.path.insert(0, str(RUNTIME))
from qcar_algorithms import source
from qcar_algorithms.cli import compatibility


def make_runtime(root, marker):
    for name in source.REQUIRED_FILES:
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(f"MARKER = {marker!r}\n", encoding="utf-8")
    return root


@pytest.fixture
def runtimes(tmp_path, monkeypatch):
    embedded = make_runtime(tmp_path / "embedded", "embedded")
    external = make_runtime(tmp_path / "external", "external")
    monkeypatch.setattr(sys, "path", list(sys.path))
    monkeypatch.setattr(sys, "meta_path", list(sys.meta_path))
    managed = source.managed_names(embedded, external) | {"only_embedded", "Observer"}
    previous = {name: module for name, module in sys.modules.items()
                if name.split(".", 1)[0] in managed}
    for name in previous:
        del sys.modules[name]
    yield embedded, external
    for name in list(sys.modules):
        if name.split(".", 1)[0] in managed:
            del sys.modules[name]
    sys.modules.update(previous)


def test_external_source_routes_runtime_and_algorithm_packages_together(runtimes):
    embedded, external = runtimes
    info = source.activate(embedded, str(external))
    assert info["root"] == str(external)
    assert importlib.import_module("vehicle_logic").MARKER == "external"
    assert importlib.import_module("Observer.local_state_estimators").MARKER == "external"
    assert importlib.import_module("Controller.controller_manager").MARKER == "external"


def test_missing_external_module_cannot_fall_back_to_embedded(runtimes):
    embedded, external = runtimes
    (embedded / "only_embedded.py").write_text("MARKER = 'wrong version'\n")
    source.activate(embedded, str(external))
    with pytest.raises(ModuleNotFoundError, match="refusing to combine"):
        importlib.import_module("only_embedded")


def test_invalid_explicit_source_does_not_change_import_paths(runtimes):
    embedded, external = runtimes
    (external / "vehicle_logic.py").unlink()
    before = list(sys.path)
    with pytest.raises(ValueError, match="Incomplete"):
        source.activate(embedded, str(external))
    assert sys.path == before


def test_already_loaded_baseline_requires_process_restart(runtimes):
    embedded, external = runtimes
    module = types.ModuleType("vehicle_logic")
    module.__file__ = str(embedded / "vehicle_logic.py")
    sys.modules["vehicle_logic"] = module
    with pytest.raises(RuntimeError, match="already imported"):
        source.activate(embedded, str(external))


def test_switch_back_cannot_claim_embedded_while_external_finder_is_active(runtimes):
    embedded, external = runtimes
    source.activate(embedded, str(external))
    with pytest.raises(RuntimeError, match="Restart Python"):
        source.activate(embedded, "embedded")


def test_default_preserves_embedded_runtime(runtimes, monkeypatch):
    embedded, _ = runtimes
    monkeypatch.delenv(source.SOURCE_ENV, raising=False)
    before = list(sys.path)
    assert source.activate(embedded)["mode"] == "embedded"
    assert sys.path == before


def test_repo_path_resolves_without_moving_algorithm_files(tmp_path):
    root = make_runtime(tmp_path / source.CRAN_RUNTIME, "cran")
    assert source.resolve_source(str(tmp_path)) == root


def test_snapshot_includes_weights_configs_and_lookup_tables_but_excludes_run_data(tmp_path):
    root = make_runtime(tmp_path, "cran")
    for name in ("Observer/models/weights.pt", "Controller/cfg/steering.csv",
                 "Observer/config.yaml", "Observer/datasets/training.npz",
                 "Observer/results/history.npz", "data_logs/drive.csv", "GUI/app.py",
                 "electronics/native/build/library.dll"):
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"example")
    names = source.snapshot(root)["files"]
    assert "Observer/models/weights.pt" in names
    assert "Observer/config.yaml" in names
    assert "Controller/cfg/steering.csv" in names
    assert not any(name.startswith(("Observer/datasets/", "Observer/results/", "GUI/", "data_logs/"))
                   or "/build/" in name for name in names)


@pytest.mark.parametrize("change", ["code", "weights", "extra", "missing"])
def test_package_rejects_changed_code_weights_added_files_and_missing_files(tmp_path, change):
    root = make_runtime(tmp_path / "runtime", "package")
    weights = root / "Observer/models/weights.pt"
    weights.parent.mkdir(parents=True)
    weights.write_bytes(b"tested weights")
    (tmp_path / "source_manifest.json").write_text(json.dumps(source.snapshot(root)))
    source.verify_package(root)
    if change == "code":
        (root / "vehicle_logic.py").write_text("MARKER = 'changed'\n")
    elif change == "weights":
        weights.write_bytes(b"different weights")
    elif change == "extra":
        (root / "extra_algorithm.py").write_text("pass\n")
    else:
        weights.unlink()
    with pytest.raises(ValueError, match="differ"):
        source.verify_package(root)


def test_ros_method_check_detects_incompatible_algorithm_runtime(tmp_path):
    root = make_runtime(tmp_path, "incompatible")
    (root / "vehicle_logic.py").write_text("class VehicleLogic:\n    pass\n")
    missing = compatibility(root)
    assert "_observer_update" in missing
    assert "_control_logic_update" in missing
