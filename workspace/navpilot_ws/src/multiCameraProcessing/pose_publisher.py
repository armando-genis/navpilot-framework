#!/usr/bin/env python3

import math
import os

import rclpy
from rclpy.node import Node
import yaml

from geometry_msgs.msg import Pose, PoseArray, Quaternion


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert roll/pitch/yaw (radians) to a geometry_msgs/Quaternion."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')

        self.declare_parameter(
            'poses_file',
            os.path.join(os.path.dirname(__file__), 'poses.yaml'),
        )
        self.declare_parameter('publish_rate', 1.0)   # Hz
        self.declare_parameter('frame_id', 'map')

        poses_file = self.get_parameter('poses_file').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.pose_array_msg = self._load_poses(poses_file)

        self.publisher_ = self.create_publisher(PoseArray, 'car_poses', 10)
        self.timer = self.create_timer(1.0 / publish_rate, self._publish)

        self.get_logger().info(
            f'Loaded {len(self.pose_array_msg.poses)} poses from {poses_file}. '
            f'Publishing on /car_poses at {publish_rate} Hz.'
        )

    def _load_poses(self, filepath: str) -> PoseArray:
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)

        msg = PoseArray()
        msg.header.frame_id = self.frame_id

        for entry in data.get('scenes', []):
            p = entry['pose']
            pose = Pose()
            pose.position.x = float(p['x'])
            pose.position.y = float(p['y'])
            pose.position.z = float(p['z'])
            pose.orientation = euler_to_quaternion(
                float(p['roll']),
                float(p['pitch']),
                float(p['heading']),
            )
            msg.poses.append(pose)

        return msg

    def _publish(self):
        self.pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.pose_array_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
