from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('tcp_gateway'),
        'config',
        'jetson1_server.yaml'
    )

    return LaunchDescription([
        Node(
            package='tcp_gateway',
            executable='tcp_gateway_node',
            name='tcp_gateway_node',
            output='screen',
            parameters=[config],
        )
    ])
