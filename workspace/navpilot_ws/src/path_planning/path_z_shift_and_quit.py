#!/usr/bin/env python3
# file: path_z_shift_node.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class PathZShiftNode(Node):
    def __init__(self):
        super().__init__('path_z_shift')

        self.z_offset = -2.2
        self.num_last_poses = 300  # number of poses to keep from the end

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(Path, '/path', self.on_path, qos)
        self.pub = self.create_publisher(Path, '/path_zshifted_and_quit', qos)

        self.get_logger().info(
            f'Listening on /path; publishing shifted last {self.num_last_poses} poses '
            f'to /path_zshifted_and_quit with z_offset={self.z_offset} m'
        )

    def on_path(self, msg: Path):
        shifted = Path()
        shifted.header = msg.header

        # Take the last N poses only
        if len(msg.poses) > self.num_last_poses:
            selected_poses = msg.poses[-self.num_last_poses:]
        else:
            selected_poses = msg.poses

        for p in selected_poses:
            q = PoseStamped()
            q.header = p.header
            q.pose = p.pose
            q.pose.position.z += self.z_offset
            shifted.poses.append(q)

        self.pub.publish(shifted)

def main():
    rclpy.init()
    node = PathZShiftNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
