import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    paramsConfig = os.path.join(get_package_share_directory('multicamera_processing'),'config','params.yaml')

    multicamera_processing_node = launch_ros.actions.Node(
        package='multicamera_processing',
        executable='multicamera_processing_node',
        name='multicamera_processing_node',
        output='screen',
        parameters=[paramsConfig],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )
    
    return launch.LaunchDescription([
        multicamera_processing_node
    ])