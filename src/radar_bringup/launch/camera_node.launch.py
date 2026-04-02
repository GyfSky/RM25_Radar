from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import Shutdown

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_detector',
            executable='camera_main',
            output='both',
            emulate_tty=True,
            respawn=True,
            respawn_delay=0.2,
        )
    ])