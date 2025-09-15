#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray
import numpy as np

class OccupancyGridTester(Node):
    def __init__(self):
        super().__init__('occupancy_grid_tester')
        
        # Subscribe to the occupancy grid
        self.occupancy_sub = self.create_subscription(
            OccupancyGrid,
            'occupancy_grid_complete_map',
            self.occupancy_callback,
            10
        )
        
        # Subscribe to the array data
        self.array_sub = self.create_subscription(
            Float64MultiArray,
            '/array',
            self.array_callback,
            10
        )
        
        self.get_logger().info('Occupancy Grid Tester started')
        self.occupancy_received = False
        self.array_received = False

    def occupancy_callback(self, msg):
        if not self.occupancy_received:
            self.get_logger().info(f'Received occupancy grid: {msg.info.width}x{msg.info.height}, resolution: {msg.info.resolution}')
            
            # Convert to numpy array for analysis
            grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            
            # Count different cell types
            free_cells = np.sum(grid == 0)
            occupied_cells = np.sum(grid == 100)
            unknown_cells = np.sum(grid == -1)
            
            self.get_logger().info(f'Grid analysis:')
            self.get_logger().info(f'  Free cells (0): {free_cells}')
            self.get_logger().info(f'  Occupied cells (100): {occupied_cells}')
            self.get_logger().info(f'  Unknown cells (-1): {unknown_cells}')
            self.get_logger().info(f'  Total cells: {grid.size}')
            
            self.occupancy_received = True

    def array_callback(self, msg):
        if not self.array_received:
            self.get_logger().info(f'Received array data with {len(msg.data)} elements')
            self.array_received = True

def main(args=None):
    rclpy.init(args=args)
    tester = OccupancyGridTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
