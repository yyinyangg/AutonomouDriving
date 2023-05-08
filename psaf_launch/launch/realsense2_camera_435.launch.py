"""Start Intel Realsense D435  Depth Camera node."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('psaf_launch'),
                          'config', 'd435i.yaml')

    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            parameters=[config],
            output='screen'
        )
    ])
