#!/usr/bin/env python3

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import launch_ros

def generate_launch_description():

    
    # Static transform: Velodyne -> Base Link (velodyne is the father of base_link)
    static_transform_velodyne_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_velodyne_to_base',
        arguments=['-1.2', '0', '-1.3', '0', '0', '0', 'velodyne', 'base_link'],
    )

    # Static transform: Base Link -> Vectornav
    static_transform_base_to_vectornav = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_base_to_vectornav',
        arguments=['0.9', '0', '1.3', '0', '0', '0', 'base_link', 'vectornav'],
    )

    # Static transform: Base Link -> Zedd
    static_transform_base_to_zedd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_base_to_zedd',
        arguments=['2.3', '0', '0', '0', '0', '0', 'base_link', 'zedd'],
    )

    # Static transform: Base Link -> Base Footprint
    static_transform_base_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_base_to_base_footprint',
        arguments=['0', '0', '-0.8', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    static_transform_base_to_light_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_base_to_base_footprint',
        arguments=['-1.93', '0.5', '-0.7', '0', '0', '0', 'base_link', 'light_1'],
    )


    static_transform_base_to_light_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_base_to_base_footprint',
        arguments=['-1.93', '-0.5', '-0.7', '0', '0', '0', 'base_link', 'light_2'],
    )
    
    return launch.LaunchDescription([
        SetEnvironmentVariable(name='RCUTILS_CONSOLE_OUTPUT_FORMAT', value='{message}'),
        static_transform_velodyne_to_base,
        static_transform_base_to_vectornav,
        static_transform_base_to_zedd,
        static_transform_base_to_base_footprint, 
        static_transform_base_to_light_1,
        static_transform_base_to_light_2,  
    ])