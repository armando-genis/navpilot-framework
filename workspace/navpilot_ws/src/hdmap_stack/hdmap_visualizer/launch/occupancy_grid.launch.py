import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    paramsConfig = os.path.join(get_package_share_directory('hdmap_visualizer'),'config','occupancy_grid_params.yaml')

    occupancy_grid_node = launch_ros.actions.Node(
        package='hdmap_visualizer',
        executable='occupancyGrip2',
        name='occupancy_grid_publisher',
        output='screen',
        parameters=[paramsConfig],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )
    
    return launch.LaunchDescription([
        occupancy_grid_node
    ])
