from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription([
        LogInfo(
            msg=(
                "Deprecated launch file 'sim_pure_pursuit_launch.py' was kept as "
                "a compatibility shim. Use an actively maintained ros2test "
                "launch file instead."
            )
        ),
    ])
