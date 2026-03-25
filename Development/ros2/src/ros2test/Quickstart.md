# QCar2 Quickstart

The current package name is `ros2test`. Older examples that use
`limo_nav_huy_test` or `limo_bringup` are obsolete for this workspace.

## Build

```bash
cd /home/nvidia/Documents/qcar2/Development/ros2
colcon build --packages-select qcar2_nodes 
colcon build --packages-select ros2test 

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
ros2 launch ros2test qcar2_slam_nav_launch_map.py
```

### Terminal 2

Run the vehicle controller separately:

```bash
cd /home/nvidia/Documents/qcar2/Development/ros2
source install/setup.bash
```

```bash
ros2 run ros2test vehicle_main_ros_qcar --ros-args -p car_id:=3 -p host:=192.168.137.1 -p v_ref:=0.6 -p vehicle_type:=Qcar
```

## Option 2: Localization only with a `.pbstream` map

Use this when you want Cartographer localization without the full Nav2 map launch.

### Terminal 1

```bash
cd /home/nvidia/Documents/qcar2/Development/ros2
source install/setup.bash
ros2 launch ros2test localization_cartographer_qcar.launch.py
  # pbstream:=/absolute/path/to/your_map.pbstream
```

### Terminal 2

```bash
cd /home/nvidia/Documents/qcar2/Development/ros2
source install/setup.bash

```

Copy-safe one-line equivalent:

```bash
ros2 run ros2test vehicle_main_ros_qcar --ros-args -p car_id:=3 -p host:=192.168.137.1 -p v_ref:=0.6 -p vehicle_type:=Qcar
```

## Notes


### Terminal 3 — Run the alignment helper
```bash
colcon build --packages-select ros2test --symlink-install

source install/setup.bash

ros2 run ros2test waypoint_alignment_helper --ros-args -p sdc_map_x:=-1.8000 -p sdc_map_y:=0.1000 -p sdc_map_z:=0.0 -p sdc_map_yaw:=1.7017 -p sdc_map_pitch:=0.0 -p sdc_map_roll:=0.0
# ros2 run ros2test waypoint_alignment_helper
```

