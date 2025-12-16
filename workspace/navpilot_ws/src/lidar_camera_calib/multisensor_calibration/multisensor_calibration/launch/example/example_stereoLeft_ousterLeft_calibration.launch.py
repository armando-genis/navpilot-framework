import os
import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    EnvironmentVariable,
)

os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "{time}: [{name}] [{severity}]\t{message}"
# Verbose log:
# os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message} ({function_name}() at {file_name}:{line_number})'

this_pkg_dir = get_package_share_directory("multisensor_calibration")
target_config_file_default = PathJoinSubstitution(
    [this_pkg_dir, "cfg/TargetWithCirclesAndAruco.yaml"]
)
robot_ws_path_default = PathJoinSubstitution(
    [EnvironmentVariable("HOME"), "multisensor_calibration/example"]
)

target_config_file = LaunchConfiguration("target_config_file")
robot_ws_path = LaunchConfiguration("robot_ws_path")

camera_sensor_name = LaunchConfiguration("camera_sensor_name")
camera_image_topic = LaunchConfiguration("camera_image_topic")
right_camera_sensor_name = LaunchConfiguration("right_camera_sensor_name")
right_camera_info_topic = LaunchConfiguration("right_camera_info_topic")
is_stereo_camera = LaunchConfiguration("is_stereo_camera")

lidar_sensor_name = LaunchConfiguration("lidar_sensor_name")
lidar_cloud_topic = LaunchConfiguration("lidar_cloud_topic")

base_frame_id = LaunchConfiguration("")
use_initial_guess = LaunchConfiguration("use_initial_guess")
save_observations = LaunchConfiguration("save_observations")
use_sim_time = LaunchConfiguration("use_sim_time")


launch_params = [
    DeclareLaunchArgument(
        name="target_config_file", default_value=target_config_file_default
    ),
    DeclareLaunchArgument(
        name="robot_ws_path",
        default_value=robot_ws_path_default,
    ),
    # Camera
    DeclareLaunchArgument(
        name="camera_sensor_name",
        default_value="stereo_camera_left",
    ),
    DeclareLaunchArgument(
        name="camera_image_topic",
        default_value="/stereo_camera/left/vis/image_color",
    ),
    DeclareLaunchArgument(
        name="right_camera_sensor_name",
        default_value="stereo_camera_right",
    ),
    DeclareLaunchArgument(
        name="right_camera_info_topic",
        default_value="/stereo_camera/right/vis/camera_info",
    ),
    DeclareLaunchArgument(
        name="is_stereo_camera",
        default_value="True",
    ),
    # Lidar
    DeclareLaunchArgument(
        name="lidar_sensor_name",
        default_value="ouster_left",
    ),
    DeclareLaunchArgument(
        name="lidar_cloud_topic",
        default_value="/ouster_left_raw_pcl/cloud",
    ),
    # Other
    DeclareLaunchArgument(
        name="base_frame_id",
        default_value="",
    ),
    DeclareLaunchArgument(
        name="use_initial_guess",
        default_value="True",
    ),
    DeclareLaunchArgument(
        name="save_observations",
        default_value="True",
    ),
    DeclareLaunchArgument(
        name="use_sim_time",
        default_value="False",
    ),
]
# Start as component:


def generate_launch_description():
    node = Node(
        namespace="calibration",
        # name=PythonExpression(["'", lidar_sensor_name, "_", ref_lidar_sensor_name, "_calibration'"]),
        package="multisensor_calibration",
        executable="extrinsic_camera_lidar_calibration",
        output="screen",
        parameters=[
            {
                "target_config_file": target_config_file,
                "robot_ws_path": robot_ws_path,
                "lidar_sensor_name": lidar_sensor_name,
                "lidar_cloud_topic": lidar_cloud_topic,
                "camera_sensor_name": camera_sensor_name,
                "camera_image_topic": camera_image_topic,
                "is_stereo_camera": is_stereo_camera,
                "right_camera_sensor_name": right_camera_sensor_name,
                "right_camera_info_topic": right_camera_info_topic,
                "base_frame_id": base_frame_id,
                "use_initial_guess": use_initial_guess,
                "save_observations": save_observations,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # run_bag = launch.actions.ExecuteProcess(
    #    cmd=['ros2', 'bag', 'play', '-l', '-r', '1.0',
    #    launch.substitutions.LaunchConfiguration('bag_path')],
    #    output='screen'
    #    )

    # rviz2 = Node(
    #     package="rviz2",
    #     namespace="",
    #     executable="rviz2",
    #     name="rviz2",
    #     arguments=["-d", str(rviz_config_path)],
    #     condition=IfCondition(launch.substitutions.LaunchConfiguration("run_rviz")),
    # )

    return launch.LaunchDescription(launch_params + [node])
