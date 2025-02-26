import os
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import Shutdown
from launch import LaunchDescription
import launch
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():

    params_config = os.path.join(get_package_share_directory('lidar_registration'), 'config', 'default.yaml')
    depth_fusion_config = os.path.join(get_package_share_directory('depth_fusion'), 'config', 'depth_fusion.yaml')
    depth_kalman_config = os.path.join(get_package_share_directory('depth_kalman'), 'config', 'depth_kalman.yaml')
    print(params_config)
    print(depth_kalman_config)

    def get_rosbag_player_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='rosbag_player_node',
            parameters=[ {'rosbag_file':
                              '/media/thesky/娱乐/bags/merged_bag_0.db3'
                          }],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_lidar_registration_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='lidar_registration_node',
            parameters=[params_config],
            extra_arguments=[{'use_intra_process_comms': True},
                             {'use_multi_threaded_executor': True}],
        )
        
    def get_depth_fusion_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='depth_fusion_node',
            parameters=[depth_fusion_config],
            extra_arguments=[{'use_intra_process_comms': True},
                             {'use_multi_threaded_executor': True}],
        )
        
    def get_depth_kalman_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='depth_kalman_node',
            parameters=[depth_kalman_config],
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
    
    def get_dynamic_cloud_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='dynamic_cloud_node',
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

    def get_convert_img_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='convert_img',
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
    ros_bag_player_node = get_rosbag_player_node('rosbag_player', 'RosbagPlayer')
    lidar_registration_node=get_lidar_registration_node('lidar_registration', 'upc_radar::LidarRegistration')
    depth_fusion_node = get_depth_fusion_node('depth_fusion', 'upc_radar::DepthFusion')
    depth_kalman_node=get_depth_kalman_node('depth_kalman','upc_radar::DepthKalman')
    kalman_filter_node=get_kalman_filter_node('kalman_filter','upc_radar::KalmanFilter')
    dynamic_cloud_node=get_dynamic_cloud_node('dynamic_cloud','upc_radar::DynamicCloud')
    cluster_node = get_cluster_node('cluster', 'upc_radar::Cluster')
    convert_img_node=get_convert_img_node('convert_img','upc_radar::ConvertImg')
    foxglove_node = get_foxglove_node('foxglove_bridge', 'foxglove_bridge::FoxgloveBridge')

    # 创建节点容器
    lidar_detector = get_container(
                                    lidar_registration_node,
                                    kalman_filter_node,
                                    dynamic_cloud_node,
                                    cluster_node,
                                    # depth_fusion_node,
                                    # depth_kalman_node,
                                    #convert_img_node,
                                    foxglove_node
                                   )
    camera_detector = Node(
        package="rps_radar24",
        executable="RadarMain"
    )
    cmd1 = launch.actions.ExecuteProcess(cmd=['ros2', 'bag', 'play', '../bags/merged_bag_0.db3', '--loop', '--start-offset', '250'])
    return LaunchDescription([
            # cmd1,
            camera_detector,
            lidar_detector
            ])
