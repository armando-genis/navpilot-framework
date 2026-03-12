#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time


class SinePublisher(Node):

    def __init__(self):
        super().__init__('sine_publisher')

        # Publisher
        self.pub = self.create_publisher(Float64, '/test/sine', 10)

        # Frequency of publishing (Hz)
        self.timer = self.create_timer(0.02, self.publish_sine)  # 50 Hz

        self.start_time = time.time()

        self.get_logger().info("Sine publisher started")

    def publish_sine(self):

        t = time.time() - self.start_time

        msg = Float64()

        # sine wave
        msg.data = float(math.sin(2.0 * math.pi * 0.5 * t))  # 0.5 Hz sine

        self.pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = SinePublisher()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
