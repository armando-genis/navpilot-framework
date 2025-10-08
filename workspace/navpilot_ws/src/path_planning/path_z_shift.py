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

        # Parameter: how much to add to z (negative to reduce)
        self.z_offset = -2.1

        # QoS: reliable + small queue (tweak if needed)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber and Publisher
        self.sub = self.create_subscription(Path, '/path', self.on_path, qos)
        self.pub = self.create_publisher(Path, '/path_zshifted', qos)

        self.get_logger().info(
            f'Listening on /path (nav_msgs/Path); publishing shifted path to /path_zshifted with z_offset={self.z_offset} m'
        )

    def on_path(self, msg: Path):
        shifted = Path()
        shifted.header = msg.header  # keep frame_id & timestamp

        # Shift each pose's Z by z_offset
        for p in msg.poses:
            q = PoseStamped()
            q.header = p.header  # keep per-pose header if present
            q.pose = p.pose      # copy pose
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
