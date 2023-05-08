import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

name = 'psaf_manual_mode'


def generate_launch_description():
    ld = LaunchDescription()
    manual_mode_node = Node(
        package=name,
        executable='manual_mode',
        name='manual_mode',
        parameters=[os.path.join(get_package_share_directory(name),
                    'config', name + '.yaml')],
        output='screen'
    )

    ld.add_action(manual_mode_node)
    return ld
