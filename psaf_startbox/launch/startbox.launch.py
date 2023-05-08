import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

name = 'psaf_startbox'


def generate_launch_description():
    ld = LaunchDescription()
    startbox_node = Node(
        package=name,
        executable='startbox',
        name='startbox',
        parameters=[os.path.join(get_package_share_directory(name),
                    'config', name + '.yaml')],
        output='screen'
    )
    ld.add_action(startbox_node)
    return ld
