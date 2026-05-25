from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("insight_ros_driver"),
        "config",
        "insight9_ros_driver.yaml",
    )

    return LaunchDescription([
        Node(
            package="insight_ros_driver",
            executable="insight9_ros_driver_node",
            name="insight9_ros_driver",
            output="screen",
            parameters=[params],
        ),
    ])
