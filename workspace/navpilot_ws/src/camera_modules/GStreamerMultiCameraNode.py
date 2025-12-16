#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


def usb_v4l2_pipeline(dev="/dev/video0", w=1280, h=720, fps=30):
    return (
        f"v4l2src device={dev} ! "
        f"video/x-raw, width={w}, height={h}, framerate={fps}/1 ! "
        f"videoconvert ! video/x-raw, format=BGR ! appsink drop=true sync=false"
    )

def obsbot_pipeline(dev="/dev/video0", w=1920, h=1080, fps=60):
    return (
        f"v4l2src device={dev} ! "
        f"video/x-raw, width={w}, height={h}, framerate={fps}/1 ! "
        f"videoconvert ! video/x-raw, format=BGR ! appsink drop=true sync=false"
    )

def obsbot_mjpg_pipeline(dev="/dev/video0", w=1920, h=1080, fps=30):
    fps = int(round(fps))
    return (
        f"v4l2src device={dev} ! "
        f"image/jpeg,width={w},height={h},framerate={fps}/1 ! "
        f"jpegdec ! videoconvert ! video/x-raw,format=BGR ! "
        f"appsink drop=true sync=false max-buffers=1"
    )

class CameraPublisher(Node):
    def __init__(self, name: str, pipeline: str, topic: str, queue_size: int, fps: float):
        super().__init__(name)
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, topic, queue_size)

        self.fps = fps
        self.period_s = 1.0 / fps

        # IMPORTANT: use CAP_GSTREAMER
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open GStreamer pipeline for {name}:\n{pipeline}")

        self.get_logger().info(f"{name} publishing → {topic} @ {fps} FPS")
        self.timer = self.create_timer(self.period_s, self.timer_cb)

    def timer_cb(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Failed to read frame (pipeline starved?)")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            if self.cap is not None:
                self.cap.release()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    QUEUE_SIZE = 1
    FPS = 30.0

    CAMERA_CONFIGS = [
        {
            "name": "obsbot",
            "topic": "racecar/camera/image_raw",
            "pipeline": obsbot_mjpg_pipeline("/dev/video0", w=1920, h=1080, fps=FPS),
        },
    ]


    nodes = []
    try:
        for cfg in CAMERA_CONFIGS:
            nodes.append(
                CameraPublisher(
                    name=cfg["name"],
                    pipeline=cfg["pipeline"],
                    topic=cfg["topic"],
                    queue_size=QUEUE_SIZE,
                    fps=FPS,
                )
            )

        while rclpy.ok():
            for n in nodes:
                rclpy.spin_once(n, timeout_sec=0.0)

    except KeyboardInterrupt:
        pass
    finally:
        for n in nodes:
            n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
