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

## Option 2: Localization only with a `.pbstream` map

Use this when you want Cartographer localization without the full Nav2 map launch.

The default launch now uses `qcar2_2d_localization_stable.lua`, which avoids
the pure-localization trimmer path that has been observed to crash on this
QCar2 Humble setup with `free(): invalid pointer`.
It also defaults to `nav2_map_server` serving the exported `.yaml` so
Cartographer does not need to publish a live occupancy grid during
saved-map localization.

### Terminal 1

```bash
cd /home/nvidia/Documents/qcar2/Development/ros2
source install/setup.bash
ros2 launch ros2test localization_cartographer_qcar.launch.py
  # pbstream:=/absolute/path/to/your_map.pbstream
  # use_static_map_server:=false  # optional: let Cartographer publish /map instead
  # cartographer_config_basename:=qcar2_2d_localization.lua  # optional: old pure-localization profile
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

ros2 run ros2test vehicle_main_ros_qcar --ros-args -p car_id:=1 -p host:=192.168.2.200 -p v_ref:=0.6 -p vehicle_type:=Qcar

## Notes


### Terminal 3 — Run the alignment helper
```bash
colcon build --packages-select ros2test --symlink-install

source install/setup.bash

ros2 run ros2test waypoint_alignment_helper --ros-args -p sdc_map_x:=0.1000 -p sdc_map_y:=-0.1000 -p sdc_map_z:=0.0 -p sdc_map_yaw:=1.5621 -p sdc_map_pitch:=0.0 -p sdc_map_roll:=0.0
# ros2 run ros2test waypoint_alignment_helper
```


cd /home/nvidia/Documents/qcar2/Development/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch qcar2_nodes qcar2_manual_cartographer_launch.py



ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/nvidia/Documents/qcar2/Development/ros2/src/ros2test/map/my_new_map.pbstream', include_unfinished_submaps: true}"


source /opt/ros/humble/setup.bash
/opt/ros/humble/lib/cartographer_ros/cartographer_pbstream_to_ros_map \
  -pbstream_filename /home/nvidia/Documents/qcar2/Development/ros2/src/ros2test/map/my_new_map.pbstream \
  -map_filestem /home/nvidia/Documents/qcar2/Development/ros2/src/ros2test/map/my_new_map \
  -resolution 0.05



## Option 0: Build a new map with Cartographer

Use the manual Cartographer launch when you want to drive the QCar2 yourself and
record a fresh map.

### Terminal 1 - Start mapping

```bash
cd /home/nvidia/Documents/qcar2/Development/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch qcar2_nodes qcar2_manual_cartographer_launch.py
```

This launch starts manual drive, LiDAR, hardware, Cartographer SLAM, and the
occupancy grid publisher. The mapping config is `qcar2_2d.lua`.

### Drive the car

Drive slowly through the whole area you want to map and, if possible, return to
your starting area once or twice so Cartographer can close loops cleanly.

### Terminal 2 - Save the `.pbstream`

After the map looks good in RViz:

```bash
cd /home/nvidia/Documents/qcar2/Development/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/nvidia/Documents/qcar2/Development/ros2/src/ros2test/map/my_new_map.pbstream', include_unfinished_submaps: true}"
```

### Export `.yaml` and `.pgm`

Convert the saved Cartographer state into a standard ROS map:

```bash
source /opt/ros/humble/setup.bash
/opt/ros/humble/lib/cartographer_ros/cartographer_pbstream_to_ros_map \
  -pbstream_filename /home/nvidia/Documents/qcar2/Development/ros2/src/ros2test/map/my_new_map.pbstream \
  -map_filestem /home/nvidia/Documents/qcar2/Development/ros2/src/ros2test/map/my_new_map \
  -resolution 0.05
```

This creates:
- `ros2/src/ros2test/map/my_new_map.pbstream`
- `ros2/src/ros2test/map/my_new_map.yaml`
- `ros2/src/ros2test/map/my_new_map.pgm`

### Use the new map later

For Cartographer localization with the saved `.pbstream`:

```bash
ros2 launch ros2test localization_cartographer_qcar.launch.py \
  pbstream:=/home/nvidia/Documents/qcar2/Development/ros2/src/ros2test/map/my_new_map.pbstream
```

The default command above uses the exported `.yaml` via `nav2_map_server` for
`/map`, while Cartographer still loads the `.pbstream` for localization.

If you want to try the older pure-localization profile anyway:

```bash
ros2 launch ros2test localization_cartographer_qcar.launch.py \
  pbstream:=/home/nvidia/Documents/qcar2/Development/ros2/src/ros2test/map/my_new_map.pbstream \
  cartographer_config_basename:=qcar2_2d_localization.lua
```

If you want Nav2 `map_server` to load the exported YAML instead:

```bash
ros2 launch ros2test localization_cartographer_qcar.launch.py \
  use_static_map_server:=true \
  map_yaml:=/home/nvidia/Documents/qcar2/Development/ros2/src/ros2test/map/my_new_map.yaml
```

If you specifically want Cartographer to publish the occupancy-grid map instead:

```bash
ros2 launch ros2test localization_cartographer_qcar.launch.py \
  use_static_map_server:=false \
  pbstream:=/home/nvidia/Documents/qcar2/Development/ros2/src/ros2test/map/my_new_map.pbstream
```

## Option 1: Full stack with Nav2 and a saved map

This launch file starts:
- QCar2 hardware nodes from `qcar2_launch.py`
- Cartographer
- Nav2 localization / navigation
- `nav2_qcar2_converter`
