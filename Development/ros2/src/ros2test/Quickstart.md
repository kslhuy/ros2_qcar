# QCar2 Quickstart

The current package name is `ros2test`. Older examples that use
`limo_nav_huy_test` or `limo_bringup` are obsolete for this workspace.

## Build

```bash
cd /home/nvidia/Documents/qcar2/Development/ros2
colcon build --packages-select qcar2_nodes 

colcon build --packages-select ros2test --symlink-install
source install/setup.bash
```

## Option 1: Full stack with Nav2 and a saved map

This launch file starts:
- QCar2 hardware nodes from `qcar2_launch.py`
- Cartographer
- Nav2 localization / navigation
- `nav2_qcar2_converter`

### Terminal 1

```bash
cd /home/nvidia/Documents/qcar2/Development/ros2
source install/setup.bash
ros2 launch ros2test qcar2_slam_nav_launch_map.py
```

### Terminal 2

Run the vehicle controller separately:

```bash
cd /home/nvidia/Documents/qcar2/Development/ros2
source install/setup.bash
ros2 run ros2test vehicle_main_ros_qcar --ros-args \
  -p car_id:=3 \
  -p host:=192.168.137.1 \
  -p v_ref:=0.6 \
  -p vehicle_type:=Qcar
```

Copy-safe one-line equivalent:

```bash
ros2 run ros2test vehicle_main_ros_qcar --ros-args -p car_id:=3 -p host:=192.168.137.1 -p v_ref:=0.6 -p vehicle_type:=Qcar
```

## Option 2: Localization only with a `.pbstream` map

Use this when you want Cartographer localization without the full Nav2 map launch.

### Terminal 1

```bash
cd /home/nvidia/Documents/qcar2/Development/ros2
source install/setup.bash
ros2 launch ros2test localization_cartographer_qcar.launch.py \
  pbstream:=/absolute/path/to/your_map.pbstream
```

### Terminal 2

```bash
cd /home/nvidia/Documents/qcar2/Development/ros2
source install/setup.bash
ros2 run ros2test vehicle_main_ros_qcar --ros-args \
  -p car_id:=3 \
  -p host:=192.168.137.1 \
  -p v_ref:=0.6 \
  -p vehicle_type:=Qcar
```

Copy-safe one-line equivalent:

```bash
ros2 run ros2test vehicle_main_ros_qcar --ros-args -p car_id:=3 -p host:=192.168.137.1 -p v_ref:=0.6 -p vehicle_type:=Qcar
```

## Notes

- For multi-line shell commands, each trailing `\` must be the last character on the line.
  Do not add spaces after `\`, or ROS may fail with `UnknownROSArgsError`.

- Do not launch `qcar2_launch.py` separately when using either launch file above.
  Those launch files already include it.
- `vehicle_main_ros_qcar` is not part of the launch files, so it must be run in
  a separate terminal.
- If the `SDCQcar -> map` alignment is wrong, override these launch arguments in
  `localization_cartographer_qcar.launch.py`:

```bash
sdc_map_x:=... sdc_map_y:=... sdc_map_z:=... \
sdc_map_yaw:=... sdc_map_pitch:=... sdc_map_roll:=...
```

- The default map for `qcar2_slam_nav_launch_map.py` is the packaged file
  `ros2test/map/my_map.yaml`.
