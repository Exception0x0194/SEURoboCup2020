import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from webots_ros2_core.webots_launcher import WebotsLauncher
from webots_ros2_core.utils import ControllerLauncher
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    webots = WebotsLauncher(
        world=os.path.join(get_package_share_directory('webots'), 'models/worlds', 'sim-robot.wbt')
    )
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
        webots,
        red_1,
        red_2,
        blue_1,
        blue_2,
        supervisor,
        Node(
            package='params',
            executable='params'
        ),
        Node(
            package='motion',
            executable='motion',
            arguments=['red_1']
        ),
        Node(
            package='motion',
            executable='motion',
            arguments=['red_2']
        ),
        Node(
            package='motion',
            executable='motion',
            arguments=['blue_1']
        ),
        Node(
            package='motion',
            executable='motion',
            arguments=['blue_2']
        ),
        launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])