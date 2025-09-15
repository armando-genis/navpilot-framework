import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('hdmap_visualizer')
    
    # Path to the occupancy grid parameters file
    occupancy_grid_params_file = os.path.join(pkg_share, 'config', 'occupancy_grid_params.yaml')

    map_visualizer_node = launch_ros.actions.Node(
        package='hdmap_visualizer',
        executable='osm_visualizer',
        name='osm_visualizer',
        parameters=[
            {"map_path": "/workspace/navpilot_ws/src/hdmap_stack/hdmap_visualizer/osms/1_new66.osm"},
            occupancy_grid_params_file
        ],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"},
        output='screen',
    )
    
    return launch.LaunchDescription([
        map_visualizer_node
    ])
