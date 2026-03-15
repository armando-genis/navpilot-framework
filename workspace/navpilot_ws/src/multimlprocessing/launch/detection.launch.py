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

    # So the node can load libonnxruntime.so.1 (used by YOLOs-CPP)
    pkg_share = get_package_share_directory('multicamera_detection')
    # .../install/multicamera_detection/share/multicamera_detection -> 4 dirnames -> workspace root
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
    onnx_lib_dir = os.path.join(
        workspace_root, 'src', 'multimlprocessing', 'YOLOs-CPP', 'onnxruntime-linux-x64-1.20.1', 'lib'
    )
    ld_library_path = os.environ.get('LD_LIBRARY_PATH', '')
    if os.path.isdir(onnx_lib_dir):
        ld_library_path = onnx_lib_dir + ((':' + ld_library_path) if ld_library_path else '')
    additional_env = {
        'RCUTILS_CONSOLE_OUTPUT_FORMAT': '{message}',
        'LD_LIBRARY_PATH': ld_library_path,
    }

    # Configs must be in the same directory as this launch file.
    # Works with: ros2 launch /path/to/multimlprocessing/launch/multiprocessing.launch.py
    # or: ros2 launch multicamera_processing multiprocessing.launch.py
    current_dir = os.path.dirname(os.path.realpath(__file__))

    ld = launch.LaunchDescription()

    detection_processing = os.path.join(current_dir, 'detection.yaml')
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
        package='multicamera_detection',
        executable='multicamera_detection_node',
        name='multicamera_detection_node',
        output='screen',
        parameters=[detection_processing],
        additional_env=additional_env,
    )

    # ld.add_action(obsbot_node)
    ld.add_action(lidar_rotation_node)
    ld.add_action(lidar_camera_matcher_node)
    ld.add_action(detection_processing_node)

    return ld

# ros2 launch /workspace/navpilot_ws/src/multimlprocessing/launch/detection.launch.py