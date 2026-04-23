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

## Option 2A: Saved-map localization with AMCL plus dead-reckoning odom

Use this as the stable localization path on the QCar2 Humble setup.
It uses a lightweight odometry node from QCar joint and IMU data to provide
`odom -> base_link`, while `nav2_map_server` and `nav2_amcl` provide saved-map
localization (`map -> odom`) from the exported `.yaml`.

This avoids the Cartographer localization path that has been observed to crash
on this platform with allocator errors such as `free(): invalid pointer`.

### Terminal 1

```bash
cd /home/nvidia/Documents/qcar2/Development/ros2
source install/setup.bash
ros2 launch ros2test localization_amcl_qcar.launch.py
  # map_yaml:=/absolute/path/to/your_map.yaml
  # publish_initial_pose:=true initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw_deg:=0.0
```

### Terminal 2

```bash
cd /home/nvidia/Documents/qcar2/Development/ros2
source install/setup.bash
```

Copy-safe one-line equivalent:

```bash
ros2 run ros2test vehicle_main_ros_qcar --ros-args -p car_id:=0 -p host:=192.168.2.200 -p v_ref:=0.6 -p vehicle_type:=Qcar
```

```bash
ros2 run ros2test vehicle_main_ros_qcar --ros-args -p car_id:=1 -p host:=192.168.2.200 -p v_ref:=0.6 -p vehicle_type:=Qcar
```

ros2 run ros2test vehicle_main_ros_qcar --ros-args -p car_id:=2 -p host:=192.168.2.200 -p v_ref:=0.6 -p vehicle_type:=Qcar


After AMCL starts, publish an initial pose in RViz or from your helper so the
`map -> odom` transform converges.

The launch now delays the AMCL/map-server stack by `2.0` seconds by default to
reduce startup scan drops while TF warms up. If you already know the map-frame
start pose, you can let the launch publish it automatically with
`publish_initial_pose:=true`.

## Option 2A+: Saved-map localization with AMCL plus RF2O and EKF

Use this when you want a better local `odom -> base_link` estimate than raw
wheel dead reckoning alone. The QCar wheel/IMU odom node is kept as a prior on
`/odom/wheel`, `rf2o_laser_odometry` adds scan-matching odom on `/odom/rf2o`,
and a local EKF fusion node publishes the live `/odom` topic and
`odom -> base_link` TF that AMCL and Nav2 consume.

The original `localization_amcl_qcar.launch.py` launch remains the fallback if
RF2O or EKF tuning is not yet stable in your environment.

The upstream `robot_localization` source is still vendored in the workspace for
future use, but it is not built by default on this QCar2 ARM Humble machine
because both GCC 9 and Clang 10 hit compiler crashes while building it.

### Terminal 1

```bash
cd /home/nvidia/Documents/qcar2/Development/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ros2test localization_amcl_rf2o_ekf_qcar.launch.py
  # map_yaml:=/absolute/path/to/your_map.yaml
```

### Terminal 2

```bash
cd /home/nvidia/Documents/qcar2/Development/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
```

If you need to go back to the simpler fallback path:

```bash
ros2 launch ros2test localization_amcl_qcar.launch.py
```

## Option 2B: Legacy Cartographer `.pbstream` localization

Keep this only for debugging or comparison with the older behavior. It loads a
saved `.pbstream` directly into Cartographer.

The default launch uses `qcar2_2d_localization_stable.lua`, which avoids the
pure-localization trimmer path that has also been observed to crash on this
QCar2 Humble setup with `free(): invalid pointer`. Even with that stability
profile, the loaded-state localization path itself may still be unstable on
this platform.

### Terminal 1

```bash
cd /home/nvidia/Documents/qcar2/Development/ros2
source install/setup.bash
ros2 launch ros2test localization_cartographer_qcar.launch.py
  # pbstream:=/absolute/path/to/your_map.pbstream
  # use_static_map_server:=true
  # map_yaml:=/absolute/path/to/your_map.yaml
```

### Terminal 2

```bash
cd /home/nvidia/Documents/qcar2/Development/ros2
source install/setup.bash
```

Copy-safe one-line equivalent:

```bash
ros2 run ros2test vehicle_main_ros_qcar --ros-args -p car_id:=0 -p host:=192.168.2.200 -p v_ref:=0.6 -p vehicle_type:=Qcar
```

```bash
ros2 run ros2test vehicle_main_ros_qcar --ros-args -p car_id:=1 -p host:=192.168.2.200 -p v_ref:=0.6 -p vehicle_type:=Qcar
```

## Notes

### Terminal 3: Run the alignment helper

```bash
colcon build --packages-select ros2test --symlink-install

source install/setup.bash

ros2 run ros2test waypoint_alignment_helper --ros-args -p sdc_map_x:=0.1000 -p sdc_map_y:=-0.1000 -p sdc_map_z:=0.0 -p sdc_map_yaw:=1.5621 -p sdc_map_pitch:=0.0 -p sdc_map_roll:=0.0
# ros2 run ros2test waypoint_alignment_helper
```

```bash
cd /home/nvidia/Documents/qcar2/Development/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch qcar2_nodes qcar2_manual_cartographer_launch.py
```

```bash
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/nvidia/Documents/qcar2/Development/ros2/src/ros2test/map/my_new_map2.pbstream', include_unfinished_submaps: false}"
```

```bash
source /opt/ros/humble/setup.bash
/opt/ros/humble/lib/cartographer_ros/cartographer_pbstream_to_ros_map \
  -pbstream_filename /home/nvidia/Documents/qcar2/Development/ros2/src/ros2test/map/my_new_map2.pbstream \
  -map_filestem /home/nvidia/Documents/qcar2/Development/ros2/src/ros2test/map/my_new_map \
  -resolution 0.05
```

## Option 0: Build a new map with Cartographer

Use the manual Cartographer launch when you want to drive the QCar2 yourself and
record a fresh map.

### Terminal 1: Start mapping

```bash
cd /home/nvidia/Documents/qcar2/Development/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch qcar2_nodes qcar2_cartographer_launch.py
```

This launch starts manual drive, LiDAR, hardware, Cartographer SLAM, and the
occupancy grid publisher. The mapping config is `qcar2_2d.lua`.

### Drive the car

Drive slowly through the whole area you want to map and, if possible, return to
your starting area once or twice so Cartographer can close loops cleanly.

### Terminal 2: Save the `.pbstream`

After the map looks good in RViz:

```bash
cd /home/nvidia/Documents/qcar2/Development/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/nvidia/Documents/qcar2/Development/ros2/src/ros2test/map/my_new_map.pbstream', include_unfinished_submaps: false}"
```

### Export `.yaml` and `.pgm`

Convert the saved Cartographer state into a standard ROS map:

```bash
source /opt/ros/humble/setup.bash
/opt/ros/humble/lib/cartographer_ros/cartographer_pbstream_to_ros_map \
  -pbstream_filename /home/nvidia/Documents/qcar2/Development/ros2/src/ros2test/map/my_new_map2.pbstream \
  -map_filestem /home/nvidia/Documents/qcar2/Development/ros2/src/ros2test/map/my_new_map \
  -resolution 0.05
```

This creates:
- `ros2/src/ros2test/map/my_new_map.pbstream`
- `ros2/src/ros2test/map/my_new_map.yaml`
- `ros2/src/ros2test/map/my_new_map.pgm`

### Use the new map later

For stable saved-map localization with AMCL:

```bash
ros2 launch ros2test localization_amcl_qcar.launch.py \
  map_yaml:=/home/nvidia/Documents/qcar2/Development/ros2/src/ros2test/map/my_new_map.yaml
```

For legacy Cartographer `.pbstream` localization:

```bash
ros2 launch ros2test localization_cartographer_qcar.launch.py \
  pbstream:=/home/nvidia/Documents/qcar2/Development/ros2/src/ros2test/map/my_new_map.pbstream \
  map_yaml:=/home/nvidia/Documents/qcar2/Development/ros2/src/ros2test/map/my_new_map.yaml
```

## Option 1: Full stack with Nav2 and a saved map

This launch file starts:
- QCar2 hardware nodes from `qcar2_launch.py`
- Cartographer
- Nav2 localization / navigation
- `nav2_qcar2_converter`


## RUN ALONE LED TRIP 
ros2 launch qcar2_nodes qcar2_led_trip_launch.py
ros2 param set /qcar2_led_trip led_color_id 3

