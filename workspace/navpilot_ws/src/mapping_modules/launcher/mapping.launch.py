import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory


from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    current_dir = os.path.dirname(os.path.realpath(__file__))
    params_path = os.path.join(current_dir, 'rotation_params.yaml')
    liosam_params = os.path.join(current_dir, 'liosam.yaml')
    rviz_config = os.path.join(current_dir, 'lio_sam.rviz')


    publisher_node_rotation = launch_ros.actions.Node(
        package='pointcloud_rotation',
        executable='pointcloud_rotation_node',
        name='pointcloud_rotation_node',
        output='screen',
        parameters=[params_path],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )

    publisher_node_lidar_imu_sync = launch_ros.actions.Node(
        package='lidar_imu_sync',
        executable='lidar_imu_fusion_node',
        name='lidar_imu_fusion_node',
        output='screen',
        parameters=[params_path],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )

    static_tf_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
        parameters=[liosam_params],
        output='screen'
    )

    imu_node = launch_ros.actions.Node(
        package='lio_sam',
        executable='lio_sam_imuPreintegration',
        name='lio_sam_imuPreintegration',
        parameters=[liosam_params],
        output='screen'
    )

    projection_node = launch_ros.actions.Node(
        package='lio_sam',
        executable='lio_sam_imageProjection',
        name='lio_sam_imageProjection',
        parameters=[liosam_params],
        output='screen'
    )

    extraction_node = launch_ros.actions.Node(
        package='lio_sam',
        executable='lio_sam_featureExtraction',
        name='lio_sam_featureExtraction',
        parameters=[liosam_params],
        output='screen'
    )

    optimization_node = launch_ros.actions.Node(
        package='lio_sam',
        executable='lio_sam_mapOptimization',
        name='lio_sam_mapOptimization',
        parameters=[liosam_params],
        output='screen'
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robot_description'),
                    'launch',
                    'display.launch.py'
                ])
            ]),
        )

    
    return launch.LaunchDescription([
        robot_launch,
        static_tf_node,
        publisher_node_rotation,
        publisher_node_lidar_imu_sync, 

        ############## LIO-SAM Nodes ##############
        imu_node,
        projection_node,
        extraction_node,
        optimization_node,
        rviz_node

    ])


