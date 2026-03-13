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
                {"video_devices": [
                    "/dev/v4l/by-path/pci-0000:05:00.3-usb-0:1.4:1.0-video-index0",
                    "/dev/v4l/by-path/pci-0000:05:00.3-usb-0:1.3:1.0-video-index0"
                ]}, 
                
                # Synchronization settings
                {"sync_enabled": False},  # True = synchronized publishing, False = independent publishing
                {"sync_tolerance_ns": 100000000},  # 100ms tolerance for USB cameras (they often have 50-100ms offset)
                {"max_queue": 10},  # Queue size per camera (should be > 1 second of frames)
                
                # These settings will be applied to all cameras:
                {"image_size": [1920, 1080]},
                {"time_per_frame": [1, 15]},
                {"pixel_format": "MJPG"},
                
                # PERFORMANCE: Publish MJPG directly without RGB conversion
                # Most ROS tools (rviz2, rqt_image_view) support MJPG natively
                {"output_encoding": "mjpeg"},  # Set to "rgb8" for uncompressed RGB
                
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
