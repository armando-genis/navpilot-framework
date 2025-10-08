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

    paramsConfig_cloud_clustering = os.path.join(current_dir, 'pointcloud_clustering.yaml')

    # obstacle clustering
    publisher_node_cloud_clustering = launch_ros.actions.LifecycleNode(
        package='pointcloud_clustering',
        namespace='',
        executable='pointcloud_clustering_node',
        name='pointcloud_clustering_node',
        output='screen',
        parameters=[paramsConfig_cloud_clustering],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )

    ld.add_action(publisher_node_cloud_clustering)

    return ld

# ros2 launch /workspace/navpilot_ws/src/lidar_modules/launch/obstacles.launch.py