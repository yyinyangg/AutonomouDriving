import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

name = 'psaf_sign_detection'


def generate_launch_description():
    ld = LaunchDescription()
    sign_detection_node = Node(
        package=name,
        executable='sign_detection',
        name='sign_detection',
        parameters=[os.path.join(get_package_share_directory(name),
                    'config', name + '.yaml')],
        output='screen'
    )
    ld.add_action(sign_detection_node)
    return ld
