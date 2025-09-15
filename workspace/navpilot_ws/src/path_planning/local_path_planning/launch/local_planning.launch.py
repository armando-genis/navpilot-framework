import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    paramsConfig = os.path.join(get_package_share_directory('local_path_planning'),'config','params.yaml')

    local = launch_ros.actions.Node(
        package='local_path_planning',
        executable='local_path_planning_node',
        name='local_path_planning_node',
        parameters=[paramsConfig],
        output='screen',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )
    
    return launch.LaunchDescription([
        local
    ])