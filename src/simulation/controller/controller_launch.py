import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from webots_ros2_core.utils import ControllerLauncher


def generate_launch_description():
    synchronization = launch.substitutions.LaunchConfiguration('synchronization', default=False)
    red_1 = ControllerLauncher(package='controller',
                                executable='controller',
                                parameters=[{'synchronization': synchronization}],
                                arguments=['red_1'],
                                output='screen')
    red_2 = ControllerLauncher(package='controller',
                                executable='controller',
                                parameters=[{'synchronization': synchronization}],
                                arguments=['red_2'],
                                output='screen')
    blue_1 = ControllerLauncher(package='controller',
                                executable='controller',
                                parameters=[{'synchronization': synchronization}],
                                arguments=['blue_1'],
                                output='screen')
    blue_2 = ControllerLauncher(package='controller',
                                executable='controller',
                                parameters=[{'synchronization': synchronization}],
                                arguments=['blue_2'],
                                output='screen')
    supervisor = ControllerLauncher(package='controller',
                                    executable='supervisor',
                                    parameters=[{'synchronization': synchronization}],
                                    output='screen')
    return LaunchDescription([
        red_1,
        red_2,
        blue_1,
        blue_2,
        supervisor
    ])