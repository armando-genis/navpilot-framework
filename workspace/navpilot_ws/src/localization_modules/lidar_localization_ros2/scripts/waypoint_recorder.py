#!/usr/bin/env python3

"""
Waypoint Recorder Node

This node subscribes to the localization pose and records waypoints as the vehicle moves.
When the node is shut down (Ctrl+C), it interpolates the recorded waypoints with 0.1m spacing
and saves them to a YAML file for path following.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
import yaml
import os
import signal
import sys
from pathlib import Path as PathLib


class WaypointRecorder(Node):
    """Records localization poses and generates interpolated waypoints."""
    
    def __init__(self):
        super().__init__('waypoint_recorder')
        
        # Parameters
        self.declare_parameter('output_file', 
            '/home/vanttec/vanttec_sdv/workspace/src/sdv_control/config/waypoints_path.yaml')
        self.declare_parameter('interpolation_distance', 0.1)  # meters
        self.declare_parameter('min_distance_threshold', 0.05)  # minimum distance to record new point
        self.declare_parameter('autosave_interval', 10.0)  # seconds
        
        self.output_file = self.get_parameter('output_file').value
        self.interp_dist = self.get_parameter('interpolation_distance').value
        self.min_dist_threshold = self.get_parameter('min_distance_threshold').value
        autosave_interval = self.get_parameter('autosave_interval').value
        
        # Storage for recorded waypoints
        self.recorded_poses = []
        self.last_position = None
        self.shutdown_requested = False
        
        # Subscriber to localization pose
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'pcl_pose',
            self.pose_callback,
            10
        )
        
        # Publishers for visualization
        self.path_pub = self.create_publisher(
            Path,
            'recorded_path',
            10
        )
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'recorded_waypoints_markers',
            10
        )
        
        self.recorded_path = Path()
        self.recorded_path.header.frame_id = 'map'
        
        # Autosave timer
        self.autosave_timer = self.create_timer(autosave_interval, self.autosave_callback)
        
        self.get_logger().info('Waypoint Recorder Node Started')
        self.get_logger().info(f'Recording waypoints. Press Ctrl+C to save and exit.')
        self.get_logger().info(f'Output file: {self.output_file}')
        self.get_logger().info(f'Interpolation distance: {self.interp_dist}m')
        self.get_logger().info(f'Autosave every {autosave_interval}s')
        
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Callback to record pose if it's far enough from the last recorded point."""
        current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        # Check if we should record this point (avoid recording duplicate points)
        if self.last_position is not None:
            distance = np.linalg.norm(current_pos - self.last_position)
            if distance < self.min_dist_threshold:
                return  # Too close to last point, skip
        
        # Record the waypoint
        self.recorded_poses.append({
            'x': float(msg.pose.pose.position.x),
            'y': float(msg.pose.pose.position.y),
            'z': float(msg.pose.pose.position.z)
        })
        
        self.last_position = current_pos
        
        # Update visualization path
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.recorded_path.poses.append(pose_stamped)
        self.recorded_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.recorded_path)
        
        # Publish markers
        self.publish_markers()
        
        if len(self.recorded_poses) % 10 == 0:
            self.get_logger().info(f'Recorded {len(self.recorded_poses)} waypoints')
    
    def interpolate_waypoints(self):
        """Interpolate waypoints with fixed distance spacing."""
        if len(self.recorded_poses) < 2:
            self.get_logger().warn('Not enough waypoints to interpolate (need at least 2)')
            return self.recorded_poses
        
        interpolated = []
        
        for i in range(len(self.recorded_poses) - 1):
            p1 = np.array([
                self.recorded_poses[i]['x'],
                self.recorded_poses[i]['y'],
                self.recorded_poses[i]['z']
            ])
            p2 = np.array([
                self.recorded_poses[i + 1]['x'],
                self.recorded_poses[i + 1]['y'],
                self.recorded_poses[i + 1]['z']
            ])
            
            # Calculate distance between points
            segment_vector = p2 - p1
            segment_length = np.linalg.norm(segment_vector)
            
            # Calculate number of interpolated points needed
            num_points = int(np.ceil(segment_length / self.interp_dist))
            
            # Generate interpolated points
            for j in range(num_points):
                t = j / num_points
                interp_point = p1 + t * segment_vector
                interpolated.append({
                    'x': float(interp_point[0]),
                    'y': float(interp_point[1]),
                    'z': float(interp_point[2])
                })
        
        # Add the last point
        interpolated.append(self.recorded_poses[-1])
        
        return interpolated
    
    def publish_markers(self):
        """Publish markers for visualization in RViz."""
        marker_array = MarkerArray()
        
        # Create sphere markers for each recorded waypoint
        for i, pose in enumerate(self.recorded_poses):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'recorded_waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = pose['x']
            marker.pose.position.y = pose['y']
            marker.pose.position.z = pose['z']
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker.lifetime.sec = 0  # 0 means forever
            
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)
    
    def autosave_callback(self):
        """Periodically save waypoints as backup."""
        if len(self.recorded_poses) > 0 and not self.shutdown_requested:
            self.get_logger().info('=' * 50)
            self.get_logger().info(f'AUTOSAVE: Saving {len(self.recorded_poses)} waypoints as backup...')
            self._save_to_file()
            self.get_logger().info('=' * 50)
    
    def save_waypoints(self):
        """Save interpolated waypoints to YAML file."""
        self.shutdown_requested = True
        
        if len(self.recorded_poses) == 0:
            self.get_logger().warn('No waypoints recorded. Nothing to save.')
            return
        
        self.get_logger().info(f'>> Processing {len(self.recorded_poses)} recorded waypoints...')
        
        # Interpolate waypoints
        interpolated = self.interpolate_waypoints()
        
        self.get_logger().info(f'>> Generated {len(interpolated)} interpolated waypoints')
        self.get_logger().info(f'>> Saving to: {self.output_file}')
        
        self._save_to_file(interpolated)
        self.get_logger().info('>> ✓ WAYPOINTS SAVED SUCCESSFULLY!')
    
    def _save_to_file(self, waypoints=None):
        """Internal method to save waypoints to file."""
        if waypoints is None:
            # Save raw waypoints (for autosave)
            waypoints = self.recorded_poses
        
        if len(waypoints) == 0:
            return
        
        # Prepare data for YAML
        waypoints_data = {
            'frame_id': 'map',
            'waypoints': waypoints
        }
        
        # Create directory if it doesn't exist
        output_path = PathLib(self.output_file)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        # Save to YAML file
        try:
            with open(self.output_file, 'w') as f:
                yaml.dump(waypoints_data, f, default_flow_style=False, sort_keys=False)
            self.get_logger().info(f'Successfully saved {len(waypoints)} waypoints to {self.output_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to save waypoints: {str(e)}')
    
    def shutdown(self):
        """Called when node is shutting down."""
        if self.shutdown_requested:
            return  # Already shutting down
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('SHUTTING DOWN WAYPOINT RECORDER')
        self.get_logger().info('=' * 60)
        self.save_waypoints()
        self.get_logger().info('=' * 60)
        self.get_logger().info('WAYPOINT RECORDER SHUTDOWN COMPLETE')
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    
    recorder = WaypointRecorder()
    
    # Setup signal handler for graceful shutdown
    def signal_handler(sig, frame):
        recorder.get_logger().info('\n========================================')
        recorder.get_logger().info('Shutdown signal received...')
        recorder.get_logger().info('========================================')
        try:
            recorder.shutdown()
        except Exception as e:
            recorder.get_logger().error(f'Error during shutdown: {str(e)}')
        try:
            recorder.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        recorder.get_logger().info('========================================')
        recorder.get_logger().info('Waypoint Recorder spinning...')
        recorder.get_logger().info('Drive the vehicle to record waypoints')
        recorder.get_logger().info('Press Ctrl+C to stop and save')
        recorder.get_logger().info('========================================')
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info('KeyboardInterrupt received')
        try:
            recorder.shutdown()
        except:
            pass
    except Exception as e:
        recorder.get_logger().error(f'Exception: {str(e)}')
        try:
            recorder.shutdown()
        except:
            pass
    finally:
        try:
            recorder.shutdown()
            recorder.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
