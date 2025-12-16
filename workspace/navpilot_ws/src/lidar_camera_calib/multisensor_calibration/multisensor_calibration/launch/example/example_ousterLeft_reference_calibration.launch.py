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

src_lidar_sensor_name = LaunchConfiguration("src_lidar_sensor_name")
src_lidar_cloud_topic = LaunchConfiguration("src_lidar_cloud_topic")
reference_name = LaunchConfiguration("reference")
reference_frame_id = LaunchConfiguration("base_link")

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
    DeclareLaunchArgument(
        name="src_lidar_sensor_name",
        default_value="ouster_rear",
    ),
    DeclareLaunchArgument(
        name="src_lidar_cloud_topic",
        default_value="/ouster_rear_raw_pcl/cloud",
    ),
    DeclareLaunchArgument(
        name="reference_name",
        default_value="ouster_left",
    ),
    DeclareLaunchArgument(
        name="reference_frame_id",
        default_value="/ouster_left_raw_pcl/cloud",
    ),
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
        # name=PythonExpression(["'", src_lidar_sensor_name, "_", reference_name, "_calibration'"]),
        package="multisensor_calibration",
        executable="extrinsic_lidar_lidar_calibration",
        output="screen",
        parameters=[
            {
                "target_config_file": target_config_file,
                "robot_ws_path": robot_ws_path,
                "src_lidar_sensor_name": src_lidar_sensor_name,
                "src_lidar_cloud_topic": src_lidar_cloud_topic,
                "reference_name": reference_name,
                "reference_frame_id": reference_frame_id,
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
