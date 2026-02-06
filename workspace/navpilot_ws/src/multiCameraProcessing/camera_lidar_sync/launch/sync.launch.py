import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    paramsConfig = os.path.join(get_package_share_directory('camera_lidar_sync'),'config','config.yaml')


    publisher_node_camera_lidar_sync = launch_ros.actions.Node(
        package='camera_lidar_sync',
        executable='camera_lidar_sync',
        name='multi_camera_sync',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"},
        output='screen',
        parameters=[paramsConfig]
    )
    
    return launch.LaunchDescription([
        publisher_node_camera_lidar_sync
    ])