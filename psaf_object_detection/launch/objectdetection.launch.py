import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

name = 'psaf_object_detection'


def generate_launch_description():
    ld = LaunchDescription()
    object_detection_node = Node(
        package=name,
        executable='object_detection',
        name='object_detection',
        parameters=[os.path.join(get_package_share_directory(name),
                    'config', name + '.yaml')],
        output='screen'
    )
    ld.add_action(object_detection_node)
    return ld
