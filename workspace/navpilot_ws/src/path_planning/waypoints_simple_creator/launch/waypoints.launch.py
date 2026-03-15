from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # waypoints_path = os.path.join(get_package_share_directory('stanley_node_pkg'),'config','waypoints.yaml')
    
    return LaunchDescription([
        Node(
            package='waypoints_simple_creator',
            executable='waypoints_creator_node',
            name='waypoints_creator_node',
            output='screen',
        )
    ])