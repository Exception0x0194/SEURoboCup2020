import sys
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    color = sys.argv[-1][7:]
    return LaunchDescription([
        Node(
            package='player',
            executable='player',
            arguments=['{}_1'.format(color)]
        ),
        Node(
            package='player',
            executable='player',
            arguments=['{}_2'.format(color)]
        )
    ])