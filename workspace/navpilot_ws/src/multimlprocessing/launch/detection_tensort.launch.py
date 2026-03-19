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

    detection_processing = os.path.join(current_dir, 'detection_tensort.yaml')
    paramsConfig_lidar_camera_matcher = os.path.join(current_dir, 'config_sync.yaml')
    paramsConfig_lidar_rotation = os.path.join(current_dir, 'config_lidar.yaml')
    paramsConfig_obsbot = os.path.join(current_dir, 'config_obsbot.yaml')

    obsbot_node = launch_ros.actions.Node(
        package='v4l2_Multicamera',
        executable='v4l2_Multicamera_node',
        name='obsbot',
        namespace='racecar/camera',
        parameters=[paramsConfig_obsbot],
        output='screen',
    )

    lidar_rotation_node = launch_ros.actions.Node(
        package='pointcloud_rotation',
        executable='pointcloud_rotation_node',
        name='pointcloud_rotation_node',
        output='screen',
        parameters=[paramsConfig_lidar_rotation],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )

    lidar_camera_matcher_node = launch_ros.actions.Node(
        package='lidar_camera_matcher',
        executable='lidar_camera_matcher',
        name='lidar_camera_matcher',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"},
        output='screen',
        parameters=[paramsConfig_lidar_camera_matcher]
    )

    detection_processing_node = launch_ros.actions.Node(
        package='multicamera_detection_tensort',
        executable='multicamera_detection_tensort_node',
        name='multicamera_detection_tensort_node',
        output='screen',
        parameters=[detection_processing],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"},
    )

    # ld.add_action(obsbot_node)
    # ld.add_action(lidar_rotation_node)
    ld.add_action(lidar_camera_matcher_node)
    ld.add_action(detection_processing_node)

    return ld

# ros2 launch /home/vanttec2/navpilot-framework/workspace/navpilot_ws/src/multimlprocessing/launch/detection_tensort.launch.py