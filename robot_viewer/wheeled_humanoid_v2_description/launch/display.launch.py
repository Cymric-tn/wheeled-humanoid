# ros2 launch wheeled_humanoid_v2_description display.launch.py   -> RViz with a slider per joint
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("wheeled_humanoid_v2_description")
    with open(os.path.join(share, "urdf", "wheeled_humanoid_v2.urdf")) as f:
        urdf = f.read()
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2"),
    ])
