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
                {"video_devices": ["/dev/video4", "/dev/video6"]}, 
                
                # Synchronization settings
                {"sync_enabled": True},  # True = synchronized publishing, False = independent publishing
                {"sync_tolerance_ns": 100000000},  # 100ms tolerance for USB cameras (they often have 50-100ms offset)
                {"max_queue": 30},  # Queue size per camera (should be > 1 second of frames)
                
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
