from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import Shutdown

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge_node',
            parameters=[{'send_buffer_limit': 1000000000}],
            output='both',
            emulate_tty=True,
            on_exit=Shutdown(),
        )
    ])