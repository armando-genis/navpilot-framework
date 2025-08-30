import numpy as np
import open3d as o3d

def create_floor_pcd(size=20.0, density=0.1, output_file="floor.pcd"):
    """
    Create a PCD file representing a floor plane.
    
    Parameters:
    - size: Size of the square floor in meters (20m x 20m)
    - density: Point density (distance between points in meters)
    - output_file: Name of the output PCD file
    """
    
    # Generate grid points
    # Create points from -size/2 to +size/2 to center the floor at origin
    x_range = np.arange(-size/2, size/2 + density, density)
    y_range = np.arange(-size/2, size/2 + density, density)
    
    # Create meshgrid
    X, Y = np.meshgrid(x_range, y_range)
    
    # Flatten the arrays and create Z array (all zeros for floor)
    x_points = X.flatten()
    y_points = Y.flatten()
    z_points = np.zeros_like(x_points)  # All Z values are 0 (floor level)
    
    # Combine into point cloud array
    points = np.column_stack((x_points, y_points, z_points))
    
    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # Optional: Add colors (gray floor)
    colors = np.tile([0.5, 0.5, 0.5], (len(points), 1))  # Gray color
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    # Save as PCD file
    o3d.io.write_point_cloud(output_file, pcd)
    
    print(f"Floor PCD created with {len(points)} points")
    print(f"Dimensions: {size}m x {size}m")
    print(f"Point density: {density}m")
    print(f"Saved as: {output_file}")
    
    return pcd

def visualize_floor_pcd(pcd_file="floor.pcd"):
    """
    Visualize the created floor PCD file.
    """
    pcd = o3d.io.read_point_cloud(pcd_file)
    o3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
    # Create floor PCD with different density options
    
    
    # Low density (0.5m spacing)
    create_floor_pcd(size=300.0, density=0.5, output_file="floor_low_density.pcd")
    
    # Visualize the high density version
    print("\nVisualizing the floor point cloud...")
    visualize_floor_pcd("floor_high_density.pcd")