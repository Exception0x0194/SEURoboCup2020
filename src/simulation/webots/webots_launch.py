import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from webots_ros2_core.webots_launcher import WebotsLauncher
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    webots = WebotsLauncher(
        world=os.path.join(get_package_share_directory('webots'), 'models/worlds', 'sim-robot.wbt')
    )
    return LaunchDescription([
        webots,
        launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])