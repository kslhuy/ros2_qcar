Implemented the offline data pipeline for Robust KalmanNet.



**Runtime Integration**
- Vehicle starts/stops recorder here: [vehicle_logic.py](c:/Users/Quang%20Huy%20Nuyen/Desktop/PHD_paper/Simulation/QCAR/QCar2_Cran/Development/multi_vehicle_self_driving_RealQcar/qcar/vehicle_logic.py#L368)
- One synchronized sample is recorded after each observer update here: [vehicle_logic.py](c:/Users/Quang%20Huy%20Nuyen/Desktop/PHD_paper/Simulation/QCAR/QCar2_Cran/Development/multi_vehicle_self_driving_RealQcar/qcar/vehicle_logic.py#L409) and [vehicle_logic.py](c:/Users/Quang%20Huy%20Nuyen/Desktop/PHD_paper/Simulation/QCAR/QCar2_Cran/Development/multi_vehicle_self_driving_RealQcar/qcar/vehicle_logic.py#L608)
- Status is exposed in telemetry here: [vehicle_logic.py](c:/Users/Quang%20Huy%20Nuyen/Desktop/PHD_paper/Simulation/QCAR/QCar2_Cran/Development/multi_vehicle_self_driving_RealQcar/qcar/vehicle_logic.py#L1006)
- Command handling for start/stop/discard/status is here: [state_base.py](c:/Users/Quang%20Huy%20Nuyen/Desktop/PHD_paper/Simulation/QCAR/QCar2_Cran/Development/multi_vehicle_self_driving_RealQcar/qcar/StateMachine/state_base.py#L872)

**GUI Trigger**
- New panel/buttons: [calibration.py](c:/Users/Quang%20Huy%20Nuyen/Desktop/PHD_paper/Simulation/QCAR/QCar2_Cran/Development/multi_vehicle_self_driving_RealQcar/qcar/GUI/qcar_gui/widgets/car_components/calibration.py#L298)
- Wired into car panel: [car_panel.py](c:/Users/Quang%20Huy%20Nuyen/Desktop/PHD_paper/Simulation/QCAR/QCar2_Cran/Development/multi_vehicle_self_driving_RealQcar/qcar/GUI/qcar_gui/widgets/car_panel.py)
- Command sender: [vehicle_commands_mixin.py](c:/Users/Quang%20Huy%20Nuyen/Desktop/PHD_paper/Simulation/QCAR/QCar2_Cran/Development/multi_vehicle_self_driving_RealQcar/qcar/GUI/qcar_gui/app_components/vehicle_commands_mixin.py)
- App callback/state wiring: [app.py](c:/Users/Quang%20Huy%20Nuyen/Desktop/PHD_paper/Simulation/QCAR/QCar2_Cran/Development/multi_vehicle_self_driving_RealQcar/qcar/GUI/qcar_gui/app.py)

**How Collection Works**
- Use `ekf` as the active local estimator during recording.
- Recorded target `x_gt` is:
  - `x, y, theta, v` from the active local estimator
  - `w` from `gyro_z`
- Recorded measurement `z` is:
  - GPS `x, y, theta` if valid, otherwise target pose fallback
  - `v` from motor tach
  - `w` from gyro
- Raw model inputs saved are:
  - `ax, ay, wz, delta, vfl, vfr, vrl, vrr`
  - wheel speeds currently use the same tach-based proxy as runtime

That keeps training and deployment consistent.

**What You Should Do Now**

1. Set local observer to `ekf` before collecting.
   - Use GUI runtime switch, or set [config_local_estimators.yaml](c:/Users/Quang%20Huy%20Nuyen/Desktop/PHD_paper/Simulation/QCAR/QCar2_Cran/Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/config_local_estimators.yaml) back to `ekf`.

2. Start the vehicle normally.

3. In the GUI, use the new `Offline RKNet Data` panel.
   - `Start`: begin recording
   - drive the car / run the scenario
   - `Save`: stop and write dataset
   - `Discard`: stop without saving

4. Find the saved dataset under:
   - `qcar/Observer/KalmaNet/Robust/datasets/`
   - file format: `*.npz`

5. Train offline with:
```powershell
python Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/KalmaNet/Robust/train_robust_kalmannet.py `
  Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/KalmaNet/Robust/datasets/your_file.npz `
  --output models/robust_kalmannet.pt `
  --epochs 30 `
  --batch-size 64 `
  --sequence-length 20
```

6. Validate baseline vs learned model with:
```powershell
python Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/KalmaNet/Robust/validate_robust_kalmannet.py `
  Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/KalmaNet/Robust/datasets/your_file.npz `
  --checkpoint models/robust_kalmannet.pt `
  --sequence-length 20 `
  --output validation_metrics.json
```

7. After that, enable learned runtime inference:
   - set `local_estimator_type: robust_kalman_net`
   - in [config_local_estimators.yaml](c:/Users/Quang%20Huy%20Nuyen/Desktop/PHD_paper/Simulation/QCAR/QCar2_Cran/Development/multi_vehicle_self_driving_RealQcar/qcar/Observer/config_local_estimators.yaml), set:
```yaml
load_pretrained: true
model_path: models/robust_kalmannet.pt
use_model: true
use_fallback: true
```

**Offline Comparison**
The validator now compares two estimators separately:
- baseline fallback estimator
- learned Robust KalmanNet estimator

Both are compared against the recorded EKF target dataset.

**Validation I Ran**
- syntax compile passed
- dataset recorder smoke test passed
- offline validator baseline-only smoke test passed

**Important**
For collection, keep the active local estimator as `ekf`. If you leave it as `robust_kalman_net`, the recorder will refuse to start by default.

**Next Steps**
1. Collect one real dataset and send me the file path; I can help tune the trainer command.
2. If you want better training quality, I can next add plots for validation trajectories and per-state RMSE curves.
3. If you want true wheel-speed training instead of tach proxy, I can patch the recorder/runtime input mapping next.

┌────────────────────────────────────────────────────────────┐
│  STEP 1: RECORD (on real QCar, via Ground Station GUI)     │
│                                                            │
│  You drive the car normally (no attacks).                  │
│  RobustKalmanNetDatasetRecorder records:                   │
│    • Raw sensors: ax, ay, wz, δ, wheel speeds              │
│    • Measurements z: GPS x/y/θ, motor tach, gyro           │
│    • Ground truth x_gt: from a trusted EKF estimator       │
│  → Saved as .npz file                                      │
└──────────────────────────┬─────────────────────────────────┘
                           ▼
┌────────────────────────────────────────────────────────────┐
│  STEP 2: TRAIN (offline, on your PC)                       │
│                                                            │
│  python train_robust_kalmannet.py dataset.npz              │
│                                                            │
│  For each batch:                                           │
│    1. Load clean windows from recorded data                │
│    2. SensorAttackAugmenter randomly corrupts branches     │
│       (bias, noise, freeze, ramp, scale, zero-out)         │
│    3. Forward: model sees CORRUPTED sensors                │
│    4. Loss: model output compared to CLEAN x_gt            │
│    5. Mask learns: suppress corrupted branch → lower loss  │
│  → Saved as .pt checkpoint                                 │
└──────────────────────────┬─────────────────────────────────┘
                           ▼
┌────────────────────────────────────────────────────────────┐
│  STEP 3: DEPLOY (back on QCar)                             │
│                                                            │
│  Load checkpoint into RobustKalmanNetStateEstimator        │
│  Now the masks can detect & suppress real sensor attacks   │
└────────────────────────────────────────────────────────────┘
 Summary: Data Collection Checklist
#	Scenario	Duration	Speed	Key Motion
1	Accel/decel straights	3 min	0→max→0	ax varies, wz≈0
2	Circles/figure-8	3 min	Steady	Constant wz, ay
3	Mixed path (turns + straights)	3 min	Varied	Transitions
4	Aggressive/edge cases	2 min	Varied	Extremes