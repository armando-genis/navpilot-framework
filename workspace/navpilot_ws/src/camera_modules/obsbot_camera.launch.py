# obsbot_camera.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="obsbot",
            namespace="racecar/camera",
            parameters=[
                {"video_device": "/dev/video0"},

                # Use the param names that work on many Humble builds:
                {"image_size": [1920, 1080]},
                {"time_per_frame": [1, 30]},

                {"pixel_format": "MJPG"},
                {"buffer_queue_size": 1},
            ],
            output="screen",
        )
    ])


# obsbot_camera.launch.py
# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package="v4l2_camera",
#             executable="v4l2_camera_node",
#             name="obsbot",
#             namespace="racecar/camera",
#             parameters=[
#                 {"video_device": "/dev/video0"},

#                 # Use the param names that work on many Humble builds:
#                 {"image_size": [640, 480]},
#                 {"time_per_frame": [1, 30]},

#                 {"pixel_format": "YUYV"},
#                 {"buffer_queue_size": 1},
#             ],
#             output="screen",
#         )
#     ])
