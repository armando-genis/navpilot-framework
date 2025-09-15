import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    paramsConfig = os.path.join(get_package_share_directory('pointcloud_clustering'),'config','pointcloud_clustering.yaml')


    publisher_node_pointcloud_clustering_node = launch_ros.actions.Node(
        package='pointcloud_clustering',
        executable='pointcloud_clustering_node',
        name='pointcloud_clustering_node',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"},
        output='screen',
        parameters=[paramsConfig]
    )
    
    return launch.LaunchDescription([
        publisher_node_pointcloud_clustering_node
    ])