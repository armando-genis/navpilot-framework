#!/usr/bin/env python3
import cv2
import threading
import time
from datetime import datetime
import numpy as np
import os

show_image = True
save_images = True  # Enable/disable image saving


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


class CameraCapture:
    def __init__(self, name: str, pipeline: str, fps: float, save_dir: str = "cameraphotos"):
        self.name = name
        self.pipeline = pipeline
        self.fps = fps
        self.period_s = 1.0 / fps
        
        self.cap = None
        self.running = False
        self.thread = None
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.frame_count = 0
        self.start_time = None
        
        # Setup save directory
        self.save_dir = os.path.join(save_dir, self.name)
        if save_images:
            os.makedirs(self.save_dir, exist_ok=True)
        
    def start(self):
        """Start the camera capture thread"""
        # IMPORTANT: use CAP_GSTREAMER
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open GStreamer pipeline for {self.name}:\n{self.pipeline}")
        
        print(f"[{self.name}] Started capture @ {self.fps} FPS")
        self.running = True
        self.start_time = time.time()
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        
    def _capture_loop(self):
        """Main capture loop running in a separate thread"""
        while self.running:
            loop_start = time.time()
            
            ok, frame = self.cap.read()
            if not ok:
                print(f"[{self.name}] Warning: Failed to read frame")
                time.sleep(self.period_s)
                continue
            
            # Store the latest frame
            with self.frame_lock:
                self.latest_frame = frame.copy()
                self.frame_count += 1
            
            # Sleep to maintain desired FPS
            elapsed = time.time() - loop_start
            sleep_time = max(0, self.period_s - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def get_frame(self):
        """Get the latest frame (thread-safe)"""
        with self.frame_lock:
            if self.latest_frame is not None:
                return self.latest_frame.copy()
            return None
    
    def get_stats(self):
        """Get capture statistics"""
        elapsed = time.time() - self.start_time if self.start_time else 0
        actual_fps = self.frame_count / elapsed if elapsed > 0 else 0
        return {
            'frame_count': self.frame_count,
            'elapsed': elapsed,
            'actual_fps': actual_fps
        }
    
    def save_frame(self):
        """Save the current frame to disk"""
        if not save_images:
            return None
        
        frame = self.get_frame()
        if frame is None:
            return None
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # milliseconds
        filename = f"{self.name}_{timestamp}.jpg"
        filepath = os.path.join(self.save_dir, filename)
        
        cv2.imwrite(filepath, frame)
        return filepath
    
    def stop(self):
        """Stop the capture thread and release resources"""
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        if self.cap is not None:
            self.cap.release()
        print(f"[{self.name}] Stopped")


def combine_frames(frames_dict, layout='horizontal'):
    """
    Combine multiple camera frames into one image
    
    Args:
        frames_dict: Dictionary of {camera_name: frame}
        layout: 'horizontal' or 'vertical' or 'grid'
    
    Returns:
        Combined image
    """
    if not frames_dict:
        return None
    
    frames = list(frames_dict.values())
    names = list(frames_dict.keys())
    
    # Add labels to each frame
    labeled_frames = []
    for name, frame in zip(names, frames):
        labeled = frame.copy()
        # Add background rectangle for text
        cv2.rectangle(labeled, (5, 5), (300, 50), (0, 0, 0), -1)
        cv2.putText(labeled, name, (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        labeled_frames.append(labeled)
    
    if layout == 'horizontal':
        # Stack horizontally
        combined = np.hstack(labeled_frames)
    elif layout == 'vertical':
        # Stack vertically
        combined = np.vstack(labeled_frames)
    elif layout == 'grid':
        # Create a grid layout
        n_cameras = len(labeled_frames)
        cols = int(np.ceil(np.sqrt(n_cameras)))
        rows = int(np.ceil(n_cameras / cols))
        
        # Resize frames to fit grid
        h, w = labeled_frames[0].shape[:2]
        grid_rows = []
        
        for i in range(rows):
            row_frames = []
            for j in range(cols):
                idx = i * cols + j
                if idx < n_cameras:
                    row_frames.append(labeled_frames[idx])
                else:
                    # Fill empty slots with black
                    row_frames.append(np.zeros_like(labeled_frames[0]))
            grid_rows.append(np.hstack(row_frames))
        
        combined = np.vstack(grid_rows)
    else:
        combined = np.hstack(labeled_frames)
    
    return combined


def main():
    CAMERA_CONFIGS = [
        {
            "name": "obsbot",
            "pipeline": obsbot_mjpg_pipeline("/dev/video0", w=1920, h=1080, fps=30),
            "fps": 30.0,
        },
        {
            "name": "obsbot2",
            "pipeline": obsbot_mjpg_pipeline("/dev/video2", w=1920, h=1080, fps=30),
            "fps": 30.0,
        },
    ]
    
    # Create combined images directory
    combined_dir = "cameraphotos/combined"
    if save_images:
        os.makedirs(combined_dir, exist_ok=True)
    
    cameras = []
    
    try:
        # Start all cameras
        print("Starting cameras...")
        for cfg in CAMERA_CONFIGS:
            cam = CameraCapture(
                name=cfg["name"],
                pipeline=cfg["pipeline"],
                fps=cfg["fps"]
            )
            cam.start()
            cameras.append(cam)
        
        print("\nAll cameras started.")
        print("Controls:")
        print("  'q' - Quit")
        print("  's' - Save current frames (individual + combined)")
        print("  'i' - Show statistics")
        print(f"\nSave directory: {os.path.abspath('cameraphotos')}")
        print()
        
        # Main display loop
        while True:
            frames_to_show = []
            
            # Get latest frame from each camera
            for cam in cameras:
                frame = cam.get_frame()
                if frame is not None:
                    # Add camera name overlay
                    cv2.putText(
                        frame, 
                        cam.name, 
                        (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        1, 
                        (0, 255, 0), 
                        2
                    )
                    frames_to_show.append((cam.name, frame))
            
            # Display frames
            if show_image and frames_to_show:
                for name, frame in frames_to_show:
                    cv2.imshow(name, frame)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("\nQuitting...")
                break
            elif key == ord('s'):
                print("\n=== Saving frames ===")
                frames_dict = {}
                
                # Save individual frames
                for cam in cameras:
                    filepath = cam.save_frame()
                    if filepath:
                        print(f"[{cam.name}] Saved: {filepath}")
                        # Get frame for combining
                        frame = cam.get_frame()
                        if frame is not None:
                            frames_dict[cam.name] = frame
                    else:
                        print(f"[{cam.name}] Failed to save frame")
                
                # Save combined frame
                if save_images and len(frames_dict) > 1:
                    combined = combine_frames(frames_dict, layout='horizontal')
                    if combined is not None:
                        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                        combined_filename = f"combined_{timestamp}.jpg"
                        combined_filepath = os.path.join(combined_dir, combined_filename)
                        cv2.imwrite(combined_filepath, combined)
                        print(f"[COMBINED] Saved: {combined_filepath}")
                
                print()
            elif key == ord('i'):
                print("\n=== Camera Statistics ===")
                for cam in cameras:
                    stats = cam.get_stats()
                    print(f"[{cam.name}] Frames: {stats['frame_count']}, "
                          f"Elapsed: {stats['elapsed']:.1f}s, "
                          f"Actual FPS: {stats['actual_fps']:.2f}")
                print()
            
            # Small sleep to prevent busy-waiting
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nCleaning up...")
        for cam in cameras:
            cam.stop()
        if show_image:
            cv2.destroyAllWindows()
        print("Done!")


if __name__ == "__main__":
    main()

