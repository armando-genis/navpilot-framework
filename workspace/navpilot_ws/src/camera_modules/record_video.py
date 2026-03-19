#!/usr/bin/env python3
"""
Multi-Camera MJPEG Viewer Node
Subscribes to multiple camera topics with MJPEG encoding and displays them using OpenCV
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class MjpegMultiCameraViewer(Node):
    def __init__(self):
        super().__init__('mjpeg_viewer')
        
        # Create CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Storage for latest frames from each camera
        self.frames = {
            0: None,
            1: None,
            2: None,
            3: None, 
            4: None
        }
        
        # Camera names for window display
        self.camera_names = {
            0: "Camera 0",
            1: "Camera 1",
            2: "Camera 2",
            3: "Camera 3",
            4: "Camera 4"
        }
        
        # Parameters
        self.declare_parameter('num_cameras', 2)
        self.declare_parameter('show_combined', True)  # Show all cameras in one window
        self.declare_parameter('show_individual', False)  # Show each camera in separate window
        self.declare_parameter('scale_factor', 0.5)  # Scale down for display (0.5 = 50%)
        self.declare_parameter('max_combined_height', 720)  # Scale combined view to fit (0 = no limit)
        self.declare_parameter('record_mp4', True)  # Save subscribed frames to MP4
        self.declare_parameter('record_fps', 30.0)  # Output FPS for VideoWriter
        # If empty, we auto-generate a filename in the current working directory.
        self.declare_parameter('output_path', '')  # e.g. "output.mp4"
        # If true, record the same combined view shown in the OpenCV window.
        # If True, also record the combined multi-camera view into a single MP4.
        self.declare_parameter('record_combined', False)
        # If True, record one MP4 per subscribed camera topic.
        self.declare_parameter('record_per_camera', True)
        # If False, per-camera MP4s use the original decoded resolution (not the resized display).
        self.declare_parameter('record_use_scaled', False)
        # Output directory for per-camera MP4s (empty = current working directory).
        self.declare_parameter('output_dir', '')  # e.g. "/tmp" or ""
        # If > 0, stop recording after this many seconds (0 = until shutdown).
        self.declare_parameter('record_duration_sec', 0.0)
        
        self.num_cameras = self.get_parameter('num_cameras').value
        self.show_combined = self.get_parameter('show_combined').value
        self.show_individual = self.get_parameter('show_individual').value
        self.scale_factor = self.get_parameter('scale_factor').value
        self.max_combined_height = self.get_parameter('max_combined_height').value
        self.record_mp4 = self.get_parameter('record_mp4').value
        self.record_fps = float(self.get_parameter('record_fps').value)
        # output_path is used only for `record_combined`.
        self.output_path = self.get_parameter('output_path').value
        self.record_combined = self.get_parameter('record_combined').value
        self.record_per_camera = self.get_parameter('record_per_camera').value
        self.record_use_scaled = self.get_parameter('record_use_scaled').value
        self.output_dir = self.get_parameter('output_dir').value
        self.record_duration_sec = float(self.get_parameter('record_duration_sec').value)

        self.video_writer = None
        self.video_started_at = None
        self.video_frame_size = None  # (w, h)

        # Per-camera recording state.
        self.camera_video_writers = {cam_id: None for cam_id in range(self.num_cameras)}
        self.camera_frame_sizes = {cam_id: None for cam_id in range(self.num_cameras)}  # (w, h)
        self.camera_record_started_at = None
        self.camera_record_ts = None
        
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)


        # Create subscribers for each camera
        self.subscribers = []
        for cam_id in range(self.num_cameras):
            topic = f'/racecar/camera/camera_{cam_id}/image_raw'
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, id=cam_id: self.image_callback(msg, id),
                qos_profile=qos
            )
            self.subscribers.append(sub)
            self.get_logger().info(f'Subscribed to {topic}')
        
        # Create display window immediately so it exists and can receive events
        if self.show_combined:
            cv2.namedWindow('Multi-Camera View', cv2.WINDOW_NORMAL)
        
        # Create a timer to update the display
        self.timer = self.create_timer(0.033, self.display_callback)  # ~30 Hz
        
        self.get_logger().info('MJPEG Multi-Camera Viewer started')
        self.get_logger().info(f'Display mode: combined={self.show_combined}, individual={self.show_individual}')
        self.get_logger().info(f'Scale factor: {self.scale_factor}, max combined height: {self.max_combined_height}')
        self.get_logger().info(
            f'Video recording: enabled={self.record_mp4}, '
            f'record_per_camera={self.record_per_camera}, record_combined={self.record_combined}, '
            f'record_use_scaled={self.record_use_scaled}, '
            f'fps={self.record_fps}, duration_sec={self.record_duration_sec}'
        )
        if self.record_mp4:
            if self.record_per_camera:
                if self.output_dir:
                    self.get_logger().info(f'Per-camera MP4 output dir: {self.output_dir}')
                else:
                    self.get_logger().info('Per-camera MP4 output dir: current working directory')
            if self.record_combined:
                if self.output_path:
                    self.get_logger().info(f'Combined MP4 output path: {self.output_path}')
                else:
                    self.get_logger().info('Combined MP4: auto-generate filename in CWD')
        self.get_logger().info('Press "q" in any window to quit')
    
    def image_callback(self, msg, camera_id):
        """Process incoming MJPEG image"""
        try:
            # Check if image is MJPEG encoded
            if msg.encoding == 'mjpeg':
                # Decode MJPEG directly from compressed bytes
                img_array = np.frombuffer(msg.data, dtype=np.uint8)
                frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                
                if frame is None:
                    self.get_logger().warn(f'Camera {camera_id}: Failed to decode MJPEG')
                    return
                    
            elif msg.encoding in ['rgb8', 'bgr8', 'mono8']:
                # Handle uncompressed formats with cv_bridge
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                self.get_logger().warn(f'Camera {camera_id}: Unsupported encoding {msg.encoding}')
                return
            
            frame_raw = frame

            # Add camera label to the raw frame so it also appears after optional resizing.
            cv2.putText(
                frame_raw,
                self.camera_names[camera_id],
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2,
                cv2.LINE_AA
            )

            # Scale only for display/combined view.
            frame_display = frame_raw
            if self.scale_factor != 1.0:
                new_width = int(frame_raw.shape[1] * self.scale_factor)
                new_height = int(frame_raw.shape[0] * self.scale_factor)
                frame_display = cv2.resize(
                    frame_raw,
                    (new_width, new_height),
                    interpolation=cv2.INTER_LINEAR
                )

            # Store the frame used for display/combined view.
            self.frames[camera_id] = frame_display

            # Optional: record one MP4 per camera topic.
            if self.record_mp4 and self.record_per_camera:
                record_frame = frame_display if self.record_use_scaled else frame_raw
                if self.camera_record_started_at is None:
                    self.camera_record_started_at = time.time()
                    self.camera_record_ts = time.strftime('%Y%m%d_%H%M%S')

                # Stop after duration if configured.
                if self.record_duration_sec > 0 and (
                    time.time() - self.camera_record_started_at
                ) >= self.record_duration_sec:
                    for cam_id in self.camera_video_writers.keys():
                        if self.camera_video_writers[cam_id] is not None:
                            self.camera_video_writers[cam_id].release()
                            self.camera_video_writers[cam_id] = None
                    self.record_mp4 = False  # stop both per-camera and combined attempts
                else:
                    self._ensure_camera_video_writer(camera_id, record_frame)
                    writer = self.camera_video_writers.get(camera_id)
                    if writer is not None:
                        size = self.camera_frame_sizes.get(camera_id)
                        if size is not None:
                            target_w, target_h = size
                            h, w = record_frame.shape[:2]
                            if (w, h) != (target_w, target_h):
                                record_frame = cv2.resize(
                                    record_frame,
                                    (target_w, target_h),
                                    interpolation=cv2.INTER_LINEAR
                                )
                        writer.write(record_frame)
            
        except Exception as e:
            self.get_logger().error(f'Camera {camera_id}: Error processing image: {str(e)}')
    
    def display_callback(self):
        """Display the latest frames"""
        try:
            combined = None
            # Show individual windows if enabled
            if self.show_individual:
                for cam_id, frame in self.frames.items():
                    if frame is not None:
                        cv2.imshow(self.camera_names[cam_id], frame)
            
            # Show combined window if enabled (always show so window exists and waitKey runs)
            if self.show_combined:
                combined = self.create_combined_view()
                if combined is not None:
                    cv2.imshow('Multi-Camera View', combined)
                else:
                    # No frames yet: show placeholder so window appears and processes events
                    placeholder = self._create_waiting_placeholder()
                    cv2.imshow('Multi-Camera View', placeholder)
            elif self.record_mp4 and self.record_combined:
                # If we're not showing the combined window but recording is enabled,
                # we still need to compute the combined frame for the encoder.
                combined = self.create_combined_view()

            # Optionally write MP4 using the same combined view.
            if self.record_mp4 and self.record_combined:
                if self.video_started_at is None:
                    self.video_started_at = time.time()

                should_stop = (
                    self.record_duration_sec > 0
                    and (time.time() - self.video_started_at) >= self.record_duration_sec
                )

                if should_stop:
                    if self.video_writer is not None:
                        self.video_writer.release()
                        self.video_writer = None
                    self.record_mp4 = False
                elif combined is not None:
                    self._ensure_video_writer(combined)
                    if self.video_writer is not None:
                        frame = combined
                        h, w = frame.shape[:2]
                        if self.video_frame_size is not None and (w, h) != self.video_frame_size:
                            target_w, target_h = self.video_frame_size
                            frame = cv2.resize(frame, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
                        self.video_writer.write(frame)
            
            # Process GUI events (longer delay so window manager can paint the window)
            key = cv2.waitKey(20) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Quit key pressed, shutting down...')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Display error: {str(e)}')
    
    def _ensure_video_writer(self, frame_bgr: np.ndarray):
        """Lazy-init VideoWriter once we know the frame size."""
        if self.video_writer is not None:
            return

        if frame_bgr is None:
            return

        h, w = frame_bgr.shape[:2]
        self.video_frame_size = (w, h)

        if self.output_path:
            out_path = self.output_path
        else:
            ts = time.strftime('%Y%m%d_%H%M%S')
            out_path = os.path.join(os.getcwd(), f'multicamera_{ts}.mp4')

        os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)

        # 'mp4v' is broadly supported by OpenCV builds.
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writer = cv2.VideoWriter(out_path, fourcc, self.record_fps, self.video_frame_size)
        if not self.video_writer.isOpened():
            self.get_logger().error(f'Failed to open VideoWriter for: {out_path}')
            self.video_writer = None
        else:
            self.get_logger().info(f'Opened MP4 writer: {out_path} @ {self.record_fps} fps')
    
    def _ensure_camera_video_writer(self, camera_id: int, frame_bgr: np.ndarray):
        """Lazy-init VideoWriter for a single camera."""
        if camera_id not in self.camera_video_writers:
            # In case num_cameras changes or a bad id is used.
            return

        if self.camera_video_writers[camera_id] is not None:
            return

        if frame_bgr is None:
            return

        h, w = frame_bgr.shape[:2]
        self.camera_frame_sizes[camera_id] = (w, h)

        out_dir = self.output_dir if self.output_dir else os.getcwd()
        if not self.camera_record_ts:
            self.camera_record_ts = time.strftime('%Y%m%d_%H%M%S')

        out_path = os.path.join(out_dir, f'camera_{camera_id}_{self.camera_record_ts}.mp4')
        os.makedirs(out_dir or '.', exist_ok=True)

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.camera_video_writers[camera_id] = cv2.VideoWriter(out_path, fourcc, self.record_fps, (w, h))
        if not self.camera_video_writers[camera_id].isOpened():
            self.get_logger().error(f'Failed to open VideoWriter for camera {camera_id}: {out_path}')
            self.camera_video_writers[camera_id] = None
        else:
            self.get_logger().info(
                f'Opened MP4 writer for camera {camera_id}: {out_path} @ {self.record_fps} fps'
            )
    
    def _create_waiting_placeholder(self, width=640, height=480):
        """Create a placeholder image when waiting for camera frames"""
        img = np.zeros((height, width, 3), dtype=np.uint8)
        img[:] = (40, 40, 40)  # Dark gray background
        text = 'Waiting for camera...'
        font = cv2.FONT_HERSHEY_SIMPLEX
        (tw, th), _ = cv2.getTextSize(text, font, 1.0, 2)
        x = (width - tw) // 2
        y = (height + th) // 2
        cv2.putText(img, text, (x, y), font, 1.0, (180, 180, 180), 2, cv2.LINE_AA)
        return img

    def create_combined_view(self):
        """Create a combined view of all cameras"""
        available_frames = [f for f in self.frames.values() if f is not None]
        
        if not available_frames:
            return None
        
        # Get dimensions from first frame
        height, width = available_frames[0].shape[:2]
        
        # Create placeholder for missing frames
        placeholder = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.putText(
            placeholder,
            'No Signal',
            (width // 4, height // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (128, 128, 128),
            2,
            cv2.LINE_AA
        )
        
        # Get all frames (use placeholder for missing ones)
        frames_to_show = []
        for cam_id in range(self.num_cameras):
            frame = self.frames.get(cam_id)
            if frame is not None:
                frames_to_show.append(frame)
            else:
                frames_to_show.append(placeholder.copy())
        
        # Arrange frames based on number of cameras
        if self.num_cameras <= 2:
            # Horizontal arrangement for 1-2 cameras
            combined = np.hstack(frames_to_show)
        else:
            # Grid arrangement for 3+ cameras
            if self.num_cameras == 3:
                # 2 on top, 1 on bottom centered
                top_row = np.hstack(frames_to_show[:2])
                bottom_frame = frames_to_show[2]
                padding = (top_row.shape[1] - bottom_frame.shape[1]) // 2
                if padding > 0:
                    left_pad = np.zeros((height, padding, 3), dtype=np.uint8)
                    right_pad = np.zeros((height, padding, 3), dtype=np.uint8)
                    bottom_row = np.hstack([left_pad, bottom_frame, right_pad])
                else:
                    bottom_row = bottom_frame
                combined = np.vstack([top_row, bottom_row])
            elif self.num_cameras == 4:
                # 2x2 grid
                top_row = np.hstack(frames_to_show[:2])
                bottom_row = np.hstack(frames_to_show[2:4])
                combined = np.vstack([top_row, bottom_row])
            elif self.num_cameras == 5:
                # 2+2+1: two rows of two, then one centered on bottom
                top_row = np.hstack(frames_to_show[:2])
                mid_row = np.hstack(frames_to_show[2:4])
                bottom_frame = frames_to_show[4]
                padding = (top_row.shape[1] - bottom_frame.shape[1]) // 2
                if padding > 0:
                    left_pad = np.zeros((height, padding, 3), dtype=np.uint8)
                    right_pad = np.zeros((height, padding, 3), dtype=np.uint8)
                    bottom_row = np.hstack([left_pad, bottom_frame, right_pad])
                else:
                    bottom_row = bottom_frame
                combined = np.vstack([top_row, mid_row, bottom_row])
            else:
                # 6+ cameras: generic grid
                rows = (self.num_cameras + 1) // 2
                cols = 2
                grid_frames = []
                for i in range(0, self.num_cameras, cols):
                    row_frames = frames_to_show[i:i+cols]
                    while len(row_frames) < cols:
                        row_frames.append(placeholder.copy())
                    grid_frames.append(np.hstack(row_frames))
                combined = np.vstack(grid_frames)
        
        # Scale down combined view to fit on screen if too tall
        if self.max_combined_height > 0 and combined.shape[0] > self.max_combined_height:
            scale = self.max_combined_height / combined.shape[0]
            new_w = int(combined.shape[1] * scale)
            new_h = int(combined.shape[0] * scale)
            combined = cv2.resize(combined, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        
        return combined
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        if hasattr(self, 'camera_video_writers'):
            for cam_id, writer in self.camera_video_writers.items():
                if writer is not None:
                    writer.release()
                    self.camera_video_writers[cam_id] = None
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        viewer = MjpegMultiCameraViewer()
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
