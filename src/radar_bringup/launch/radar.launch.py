import os
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import Shutdown
from launch import LaunchDescription
import launch
import numpy as np
from ament_index_python.packages import get_package_share_directory
from tf2_geometry_msgs.tf2_geometry_msgs import _decompose_affine

def get_matrix_tf_broadcaster(cali: np.array, fr: str, child_fr: str):
    quat, trans = _decompose_affine(cali)
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace='radar',
        name=fr+'_to_'+child_fr,
        arguments=['--x', str(trans[0]),
                   '--y', str(trans[1]),
                   '--z', str(trans[2]),
                   '--qw', str(quat[0]),
                   '--qx', str(quat[1]),
                   '--qy', str(quat[2]),
                   '--qz', str(quat[3]),
                   '--frame-id', fr,
                   '--child-frame-id', child_fr],)

def generate_launch_description():

    params_config = os.path.join(get_package_share_directory('radar_bringup'), 'config', 'default.yaml')
    print(params_config)

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
            # on_exit=Shutdown(),
            respawn=True,
            respawn_delay=0.2,
        )
    def read_matrix_from_file(file_path):
        with open(file_path, "r") as f:
            lines = f.readlines()

        matrix_data = []
        i=0;
        for line in lines:
            if i==4:
                break
            row_data = [float(x) for x in line.strip().split()]
            matrix_data.append(row_data)
            i=i+1

        return np.array(matrix_data)

    cali_matrix = read_matrix_from_file("/home/thesky/RM25_Radar/resource/lidar2world.txt")
        
    
    # 创建节点描述
    lidar_registration_node=get_lidar_registration_node('lidar_registration', 'upc_radar::LidarRegistration')
    kalman_filter_node=get_kalman_filter_node('kalman_filter','upc_radar::KalmanFilter')
    dynamic_cloud_node=get_dynamic_cloud_node('dynamic_cloud','upc_radar::DynamicCloud','')
    mid70_dynamic_cloud_node=get_dynamic_cloud_node('dynamic_cloud','upc_radar::DynamicCloud','mid70')
    avia_dynamic_cloud_node=get_dynamic_cloud_node('dynamic_cloud','upc_radar::DynamicCloud','avia')
    cluster_node = get_cluster_node('cluster', 'upc_radar::Cluster')
    foxglove_node = get_foxglove_node('foxglove_bridge', 'foxglove_bridge::FoxgloveBridge')
    lidar_tf=get_matrix_tf_broadcaster(
        np.array([[0.931098 , -0.36456 ,-0.0123815 ,-0.0245926],
                  [0.364407  , 0.928117 , 0.0762062 ,-0.0841121],
                  [-0.0162903 , -0.0754673 ,  0.997015 ,-0.00709805],
                  [0.,0.,0.,1.],]), 'lidar_avia_frame', 'lidar_mid70_frame'),
    lidar_extrinsic_tf=get_matrix_tf_broadcaster(cali_matrix, 'rm_frame', 'lidar_avia_frame'),

    # 创建节点容器
    lidar_detector = get_container(
                                    kalman_filter_node,
                                    mid70_dynamic_cloud_node,
                                    avia_dynamic_cloud_node,
                                    foxglove_node
                                   )
    camera_detector = Node(
        package="rps_radar24",
        executable="RadarMain",
        output='both',
        emulate_tty=True,
        respawn=True,
        respawn_delay=0.2,
    )

    return LaunchDescription([
            *lidar_tf,
            *lidar_extrinsic_tf,
            camera_detector,
            lidar_detector
            ])
