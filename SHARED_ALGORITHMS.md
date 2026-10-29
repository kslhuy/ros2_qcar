# Shared algorithm workflow

`QCar2_Cran/Development/multi_vehicle_self_driving_RealQcar/qcar` is the
development source. The QCar ROS entry point can now use that **whole runtime**:
observers, controllers, path planning, state machines, V2V/attack logic,
calibration/system identification, taxi logic, perception, and their support
modules. The matching factories and configuration travel with the algorithms.

The ROS node, ROS messages, localization, coordinate transforms, and
`QcarAdaptRos` hardware adapters remain in `ros2_qcar`. The original embedded
runtime remains available as a baseline. No existing algorithm files in either
project have been overwritten or merged.

## Local development

Open `qcar-development.code-workspace` in VS Code. Both projects appear in the
Explorer. **New integrated terminals** set `QCAR_ALGORITHM_SOURCE` to the
`QCar2_Cran` folder. Restart a vehicle process to pick up code changes.

From a normal PowerShell terminal in `ros2_qcar`:

```powershell
python tools/qcar_algorithms.py check --source ../QCar2_Cran
python tools/qcar_algorithms.py compare --source ../QCar2_Cran
$env:QCAR_ALGORITHM_SOURCE = (Resolve-Path ../QCar2_Cran).Path
```

From Bash on a machine with both repositories:

```bash
export QCAR_ALGORITHM_SOURCE=/absolute/path/to/QCar2_Cran
```

Rebuild the ROS package once after installing this connector:

```bash
cd Development/ros2
colcon build --packages-select ros2test --symlink-install
source install/setup.bash
```

Then use your normal ROS launch commands from `Quickstart.md`. The
`vehicle_main_ros_qcar` entry point prints `[ALGORITHMS]` with the selected
directory and source commit before loading `VehicleLogic`. Its default vehicle
fleet configuration is still discovered in the ROS package. Algorithm-specific
YAML, calibration tables, and model weights come from the selected source.
For example, the current embedded local observer default is `high_gain_observer`
while the current Cran default is `ekf`; source selection intentionally changes
the algorithm defaults too. Review them when switching an experiment.

You can also launch a command with source selection scoped to just that process:

```bash
python tools/qcar_algorithms.py run --source /path/to/QCar2_Cran -- ros2 run ros2test vehicle_main_ros_qcar --ros-args -p car_id:=0
```

That command starts the normal vehicle runtime. `check` and `compare` only read
files; they do not start ROS or hardware. `check` verifies source completeness
and methods called by the ROS wrapper. It is a static compatibility check, not
a check of controller behavior, dependencies, timing, or physical calibration.

## Versioned hardware deployment

After testing an algorithm version, build an installable snapshot from that
same working tree. Use a Python environment containing `pip`, `setuptools>=65`,
and `wheel`. The build uses local tools and does not download dependencies:

```powershell
python tools/qcar_algorithms.py package --source ../QCar2_Cran
```

The wheel is written to `dist/algorithms/`. Its version includes the source
commit and a fingerprint of the exported contents. `source_manifest.json`
records hashes of every exported code, configuration, and model file, including
uncommitted changes. Logs, recorded datasets, papers, GUIs, and native build
products are excluded. Native source is included; platform-specific native
libraries must be built separately on the target. The original files stay in
`QCar2_Cran`.

External perception resources also remain separate: for example, SCNN uses
`Development/pytorch_auto_drive`, and some lane detector defaults point to
Quanser's pretrained-model directory. Their code is shared, but those external
libraries/model files still need to exist at the target's configured paths.

Copy the generated wheel to the QCar, then install it in the **same Python
environment used by ROS**:

```bash
python3 -m pip install --no-deps /path/to/qcar_algorithms-<version>-py3-none-any.whl
export QCAR_ALGORITHM_SOURCE=package
qcar-algorithms check --source package
```

Use the exact wheel filename produced by the build. The target must already
have the algorithms' dependencies, including the Quanser SDK and any selected
optional ML/MPC/vision dependencies. The wheel does not install ROS, the SDK,
PyTorch, or compiled native observer libraries automatically.

Run your normal ROS commands. Startup verifies the packaged files and prints
the package fingerprint. A missing source or changed package fails explicitly.
The loader prevents combining missing top-level runtime modules with the
embedded copy and rejects changing sources after runtime imports.

## Return to the embedded baseline

In a fresh vehicle process, select the original ROS copy:

```powershell
$env:QCAR_ALGORITHM_SOURCE = 'embedded'
```

```bash
export QCAR_ALGORITHM_SOURCE=embedded
```

An ordinary terminal with no `QCAR_ALGORITHM_SOURCE` also uses the embedded
baseline. The workspace's terminal setting is local development convenience;
it does not change the environment on a remote vehicle.
