#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')

        # Create publisher
        self.publisher_ = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # Publish at 30 Hz
        timer_period = 1.0 / 30.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Pre-fill CameraInfo message with your calibration
        self.camera_info = CameraInfo()

        # TODO: set to your real resolution
        self.camera_info.width = 1280
        self.camera_info.height = 720

        self.camera_info.header.frame_id = 'camera_optical_frame'

        # Distortion model and coefficients
        self.camera_info.distortion_model = 'plumb_bob'
        self.camera_info.d = [
            0.11208136,    # k1
            -0.82520621,   # k2
            0.00725262,    # p1
            -0.00195751,   # p2
            1.16303315     # k3
        ]

        # Intrinsic camera matrix K (3x3) in row-major order
        self.camera_info.k = [
            1092.83795872, 0.0,           631.80024334,
            0.0,           1091.39567552, 349.32490162,
            0.0,           0.0,           1.0
        ]

        # Rectification matrix R (identity for monocular camera)
        self.camera_info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        # Projection matrix P (3x4)
        self.camera_info.p = [
            1092.83795872, 0.0,           631.80024334, 0.0,
            0.0,           1091.39567552, 349.32490162, 0.0,
            0.0,           0.0,           1.0,          0.0
        ]

        self.get_logger().info('CameraInfoPublisher node has been started.')

    def timer_callback(self):
        # Update timestamp
        self.camera_info.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.camera_info)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
