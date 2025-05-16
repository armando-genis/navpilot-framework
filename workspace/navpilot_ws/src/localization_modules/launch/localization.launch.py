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
        publisher_node_rotation,
        publisher_node_lidar_imu_sync, 

    ])


