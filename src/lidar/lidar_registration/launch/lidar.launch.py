import os
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import Shutdown
from launch import LaunchDescription
import launch

def generate_launch_description():
    def get_localization_hits_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='lidar_registration_node',
            extra_arguments=[{'use_intra_process_comms': True},
                             {'use_multi_threaded_executor': True}],
        )
        

    def get_foxglove_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='foxglove_bridge_node',
            parameters=[ {'send_buffer_limit': 1000000000}],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_container(*nodes):
        # 打印当前路径
        print(os.getcwd())
        return ComposableNodeContainer(
            name='lidar_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=list(nodes),
            output='both',
            emulate_tty=True,
            on_exit=Shutdown(),
        )
        
    
    # 创建节点描述
    localization_hits_node=get_localization_hits_node('lidar_registration', 'upc_radar::Lidar_Registration')
    foxglove_node = get_foxglove_node('foxglove_bridge', 'foxglove_bridge::FoxgloveBridge')

    # 创建节点容器
    lidar_detector = get_container(
                                    localization_hits_node,
                                    foxglove_node
                                   )
    cmd1 = launch.actions.ExecuteProcess(cmd=['ros2', 'bag', 'play', '../../bags/merged_bag_0.db3', '--loop', '--start-offset', '250'])
    return LaunchDescription([
            # cmd1,
            lidar_detector
            ])