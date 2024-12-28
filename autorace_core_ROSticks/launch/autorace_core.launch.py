from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autorace_core_ROSticks',
            executable='controller',
            name='autorace_core_ROSticks_controller'
        )
    ])
