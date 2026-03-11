colcon build --packages-select ros2test && source install/setup.bash
ros2 launch ros2test qcar2_slam_nav_launch_map.py

ros2 run ros2test vehicle_main_ros_qcar --ros-args \
  -p car_id:=3 \
  -p v_ref:=0.6 \
  -p vehicle_type:=Limo