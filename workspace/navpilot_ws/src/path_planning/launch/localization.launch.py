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

    rviz_param_dir = os.path.join(current_dir, 'localization.rviz')
    params_rotation_ = os.path.join(current_dir, 'rotation_params.yaml')
    paramsConfig_cloud_clustering = os.path.join(current_dir, 'pointcloud_clustering.yaml')
    params_routing = os.path.join(current_dir, 'routing_params.yaml')
    params_path_planning = os.path.join(current_dir, 'path_planning.yaml')


    publisher_node_rotation = launch_ros.actions.LifecycleNode(
        package='pointcloud_rotation',
        namespace='',
        executable='pointcloud_rotation_node',
        name='pointcloud_rotation_node',
        output='screen',
        parameters=[params_rotation_],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )

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

    publisher_node_occupancy = launch_ros.actions.LifecycleNode(
        package='map_visualizer',
        namespace='',
        executable='occupancy_pub',
        name='occupancy_pub',
        output='screen',
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

    # obstacle clustering
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_description'),
                'launch',
                'sdv_localization.launch.py'
            ])
        ]),
    )


    # paht planning
    publisher_node_path_planning = launch_ros.actions.LifecycleNode(
        package='local_path_planning',
        namespace='',
        executable='local_path_planning_node',
        name='local_path_planning_node',
        output='screen',
        parameters=[params_path_planning],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )

    # yaw test
    publisher_node_yaw_test = launch_ros.actions.LifecycleNode(
        package='yaw_test',
        namespace='',
        executable='yaw_car_test',
        name='yaw_car_test',
        output='screen',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )

    static_map_to_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_velodyne',
        arguments=['0', '0', '2.1', '0', '0', '0', 'map', 'velodyne'],
        output='screen'
    )

    # rviz = launch_ros.actions.Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', rviz_param_dir]
    # )

    ld.add_action(robot_launch)
    ld.add_action(publisher_node_rotation)
    ld.add_action(publisher_node_osm)
    ld.add_action(publisher_node_routing)
    # ld.add_action(rviz) 
    ld.add_action(publisher_node_occupancy)
    ld.add_action(static_map_to_velodyne)
    ld.add_action(publisher_node_yaw_test)
    ld.add_action(publisher_node_path_planning)
    ld.add_action(publisher_node_cloud_clustering)

    return ld