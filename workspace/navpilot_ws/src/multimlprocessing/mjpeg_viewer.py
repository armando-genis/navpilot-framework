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
        self.declare_parameter('num_cameras', 3)
        self.declare_parameter('show_combined', True)  # Show all cameras in one window
        self.declare_parameter('show_individual', False)  # Show each camera in separate window
        self.declare_parameter('scale_factor', 0.5)  # Scale down for display (0.5 = 50%)
        self.declare_parameter('max_combined_height', 720)  # Scale combined view to fit (0 = no limit)
        
        self.num_cameras = self.get_parameter('num_cameras').value
        self.show_combined = self.get_parameter('show_combined').value
        self.show_individual = self.get_parameter('show_individual').value
        self.scale_factor = self.get_parameter('scale_factor').value
        self.max_combined_height = self.get_parameter('max_combined_height').value
        
        # Create subscribers for each camera
        self.subscribers = []
        for cam_id in range(self.num_cameras):
            topic = f'/racecar/camera/camera_{cam_id}/image_raw'
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, id=cam_id: self.image_callback(msg, id),
                10
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
            
            # Scale frame if needed
            if self.scale_factor != 1.0:
                new_width = int(frame.shape[1] * self.scale_factor)
                new_height = int(frame.shape[0] * self.scale_factor)
                frame = cv2.resize(frame, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
            
            # Add camera label to frame
            cv2.putText(
                frame, 
                self.camera_names[camera_id],
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2,
                cv2.LINE_AA
            )
            
            # Store the frame
            self.frames[camera_id] = frame
            
        except Exception as e:
            self.get_logger().error(f'Camera {camera_id}: Error processing image: {str(e)}')
    
    def display_callback(self):
        """Display the latest frames"""
        try:
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
            
            # Process GUI events (longer delay so window manager can paint the window)
            key = cv2.waitKey(20) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Quit key pressed, shutting down...')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Display error: {str(e)}')
    
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
