# obsbot_camera.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="v4l2_Multicamera",
            executable="v4l2_Multicamera_node",
            name="obsbot",
            namespace="racecar/camera",
            parameters=[
                # Multi-camera mode: provide list of video devices
                # If you have more than one device, multi-camera mode will be activated
                {"video_devices": ["/dev/video0", "/dev/video2"]},  # Add more devices as needed
                
                # Optional: synchronization settings (defaults shown)
                {"sync_tolerance_ns": 10000000},  # 10ms tolerance for frame synchronization
                {"max_queue": 10},  # Maximum queue size per camera
                
                # These settings will be applied to all cameras:
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
