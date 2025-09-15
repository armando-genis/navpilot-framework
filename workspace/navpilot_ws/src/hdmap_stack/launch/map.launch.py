import os

import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    current_dir = os.path.dirname(os.path.realpath(__file__))
    ld = launch.LaunchDescription()
    params_routing = os.path.join(current_dir, 'routing_params.yaml')

    # ros2 launch map_visualizer osm_visualizer.launch.py
    publisher_node_osm = launch_ros.actions.LifecycleNode(
        package='map_visualizer',
        namespace='',
        executable='osm_visualizer',
        name='osm_visualizer',
        output='screen',
        parameters=[params_routing],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )
    publisher_node_routing = launch_ros.actions.LifecycleNode(
        package='waypoints_routing',
        namespace='',
        executable='waypoints_routing_node',
        name='waypoints_routing_node',
        output='screen',
        parameters=[params_routing],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )

    # occupancy grid
    publisher_node_occupancy = launch_ros.actions.LifecycleNode(
        package='map_visualizer',
        namespace='',
        executable='occupancy_pub',
        name='occupancy_pub',
        output='screen',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )

    ld.add_action(publisher_node_osm)
    ld.add_action(publisher_node_routing)
    ld.add_action(publisher_node_occupancy)

    return ld

# ros2 launch /workspace/navpilot_ws/src/hdmap_stack/launch/map.launch.py