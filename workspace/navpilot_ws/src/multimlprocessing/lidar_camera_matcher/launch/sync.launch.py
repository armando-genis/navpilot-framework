import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    paramsConfig = os.path.join(get_package_share_directory('lidar_camera_matcher'),'config','config.yaml')


    lidar_camera_matcher_node = launch_ros.actions.Node(
        package='lidar_camera_matcher',
        executable='lidar_camera_matcher',
        name='lidar_camera_matcher',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"},
        output='screen',
        parameters=[paramsConfig]
    )
    
    return launch.LaunchDescription([
        lidar_camera_matcher_node
    ])