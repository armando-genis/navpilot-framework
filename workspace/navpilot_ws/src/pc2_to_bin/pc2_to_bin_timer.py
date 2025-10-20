#!/usr/bin/env python3
import os
import sys
import argparse
import signal
from pathlib import Path
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


class PointCloudSaver(Node):
    def __init__(self, topic: str, out_dir: Path, period_sec: float):
        super().__init__("pc2_to_bin_saver")

        self.out_dir = out_dir
        self.period_sec = period_sec
        self.counter = 1
        self.latest_msg: Optional[PointCloud2] = None
        self.has_new_data = False

        # Ensure output directory exists
        self.out_dir.mkdir(parents=True, exist_ok=True)

        # Sensor data QoS (reliable or best-effort both common; use sensor-like QoS)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub = self.create_subscription(
            PointCloud2, topic, self._pc_cb, qos
        )
        self.timer = self.create_timer(self.period_sec, self._on_timer)

        self.get_logger().info(
            f"Subscribed to '{topic}'. Saving every {self.period_sec:.1f}s to '{self.out_dir}'."
        )

    def _pc_cb(self, msg: PointCloud2):
        # Keep the most recent message and mark it as new
        self.latest_msg = msg
        self.has_new_data = True

    def _on_timer(self):
        if self.latest_msg is None:
            self.get_logger().warn("No PointCloud2 received yet; skipping this interval.")
            return

        if not self.has_new_data:
            self.get_logger().warn("No new PointCloud2 data since last save; skipping this interval.")
            return

        try:
            arr = self._pc2_to_kitti_array(self.latest_msg)
            if arr.size == 0:
                self.get_logger().warn("Received cloud has 0 valid points; skipping.")
                return

            fname = self.out_dir / f"{self.counter:06d}.bin"
            # Write as float32 binary in row-major order
            arr.astype(np.float32, copy=False).tofile(fname)
            self.get_logger().info(
                f"Saved {arr.shape[0]} points to {fname.name}"
            )
            self.counter += 1
            self.has_new_data = False  # Mark data as saved
        except Exception as e:
            self.get_logger().error(f"Failed to save .bin: {e}")

    def _pc2_to_kitti_array(self, msg: PointCloud2) -> np.ndarray:
        names = [f.name for f in msg.fields]
        intensity_field = next((n for n in ["intensity", "i", "intensities"] if n in names), None)

        if intensity_field:
            it = pc2.read_points(msg, field_names=["x","y","z", intensity_field], skip_nans=True)
            rec = np.fromiter(it, dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('i','<f4')], count=-1)
            # reinterpret the structured array as a plain float32 array
            pts = rec.view('<f4').reshape(-1, 4)
        else:
            it = pc2.read_points(msg, field_names=["x","y","z"], skip_nans=True)
            rec = np.fromiter(it, dtype=[('x','<f4'),('y','<f4'),('z','<f4')], count=-1)
            if rec.size == 0:
                return np.empty((0, 4), dtype=np.float32)
            xyz = rec.view('<f4').reshape(-1, 3)
            pts = np.hstack((xyz, np.zeros((xyz.shape[0], 1), dtype=np.float32)))

        return pts.astype(np.float32, copy=False)

def main():

    topic = "/velodyne_points"
    out_dir = Path("./bins")
    period = 3.0

    rclpy.init(args=None)
    node = PointCloudSaver(topic, out_dir, period)

    # Graceful shutdown on Ctrl+C / SIGTERM
    def _shutdown_handler(signum, frame):
        node.get_logger().info("Shutting down...")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown_handler)
    signal.signal(signal.SIGTERM, _shutdown_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
