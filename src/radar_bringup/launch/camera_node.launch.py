from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import Shutdown

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rps_radar24',
            executable='RadarMain',
            output='both',
            emulate_tty=True,
            respawn=True,
            respawn_delay=0.2,
        )
    ])