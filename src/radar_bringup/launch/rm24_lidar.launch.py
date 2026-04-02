import os
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import Shutdown
from launch import LaunchDescription
import launch
import numpy as np
from ament_index_python.packages import get_package_share_directory
from tf2_geometry_msgs.tf2_geometry_msgs import _decompose_affine

def generate_launch_description():

    params_config = os.path.join(get_package_share_directory('radar_bringup'), 'config', 'rm24_config.yaml')

    def get_lidar_registration_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='lidar_registration_node',
            parameters=[params_config],
            extra_arguments=[{'use_intra_process_comms': True},
                             {'use_multi_threaded_executor': True}],
        )

    def get_kalman_filter_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='kalman_filter_node',
            parameters=[params_config],
            extra_arguments=[{'use_intra_process_comms': True},
                             {'use_multi_threaded_executor': True}],
        )
    def get_dynamic_cloud_node(package, plugin,ns):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='dynamic_cloud_node',
            namespace=ns,
            parameters=[params_config],
            extra_arguments=[{'use_intra_process_comms': True},
                             {'use_multi_threaded_executor': True}],
        )
    
    def get_cluster_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='cluster_node',
            parameters=[params_config],
            extra_arguments=[{'use_intra_process_comms': True},
                             {'use_multi_threaded_executor': True}]
        )

    def get_foxglove_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='foxglove_bridge_node',
            parameters=[ {'send_buffer_limit': 1000000000}],
            extra_arguments=[{'use_intra_process_comms': True},
                             {'use_multi_threaded_executor': True}]
        )

    def get_container(*nodes):
        # 打印当前路径
        print(os.getcwd())
        return ComposableNodeContainer(
            name='my_lidar_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            arguments=['--use_multi_threaded_executor'],
            composable_node_descriptions=list(nodes),
            output='both',
            emulate_tty=True,
            on_exit=Shutdown(),
        )
        
    
    # 创建节点描述
    lidar_registration_node=get_lidar_registration_node('lidar_registration', 'upc_radar::LidarRegistration')
    kalman_filter_node=get_kalman_filter_node('kalman_filter','upc_radar::KalmanFilter')
    dynamic_cloud_node=get_dynamic_cloud_node('dynamic_cloud','upc_radar::DynamicCloud','')
    cluster_node = get_cluster_node('cluster', 'upc_radar::Cluster')
    foxglove_node = get_foxglove_node('foxglove_bridge', 'foxglove_bridge::FoxgloveBridge')

    # 创建节点容器
    lidar_detector = get_container(
                                    lidar_registration_node,
                                    kalman_filter_node,
                                    dynamic_cloud_node,
                                    foxglove_node
                                   )
    camera_detector = Node(
        package="camera_detector",
        executable="camera_main"
    )

    return LaunchDescription([
            # camera_detector,
            lidar_detector
            ])
