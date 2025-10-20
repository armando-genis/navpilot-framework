#!/usr/bin/env python3
import time
import struct
from pathlib import Path

import numpy as np
import open3d as o3d


def load_kitti_bin(bin_path: Path) -> o3d.geometry.PointCloud:
    """Load a KITTI-format .bin file into an Open3D PointCloud."""
    arr = np.fromfile(bin_path, dtype=np.float32)
    if arr.size % 4 != 0:
        raise ValueError(f"{bin_path} has {arr.size} floats (not divisible by 4)")
    pts = arr.reshape(-1, 4)
    xyz = pts[:, :3]
    intensity = pts[:, 3]

    # Map intensity to gray (0–1)
    norm_intensity = (intensity - intensity.min()) / (intensity.ptp() + 1e-8)
    colors = np.stack([norm_intensity]*3, axis=1)

    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(xyz)
    pc.colors = o3d.utility.Vector3dVector(colors)
    return pc


def create_coordinate_frame(size=10.0):
    """Create a coordinate frame (X=red, Y=green, Z=blue)."""
    return o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)


def create_grid(size=50.0, spacing=5.0):
    """Create a ground grid for reference."""
    grid_lines = []
    
    # Create grid lines
    for i in range(int(-size/2), int(size/2) + 1, int(spacing)):
        # Lines parallel to X axis
        grid_lines.append([[i, -size/2, 0], [i, size/2, 0]])
        # Lines parallel to Y axis  
        grid_lines.append([[-size/2, i, 0], [size/2, i, 0]])
    
    # Create line set
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(np.array(grid_lines).reshape(-1, 3))
    line_set.lines = o3d.utility.Vector2iVector(np.arange(len(grid_lines) * 2).reshape(-1, 2))
    line_set.colors = o3d.utility.Vector3dVector([[0.5, 0.5, 0.5]] * len(grid_lines) * 2)  # Gray color
    
    return line_set


def create_visualization_geometries(pc):
    """Create all geometries needed for visualization with reference frames."""
    geometries = [pc]
    
    # Add coordinate frame at origin
    coord_frame = create_coordinate_frame(size=5.0)
    geometries.append(coord_frame)
    
    # Add ground grid
    grid = create_grid(size=50.0, spacing=5.0)
    geometries.append(grid)
    
    return geometries


def main():
    bins_dir = Path("./bins")
    if not bins_dir.exists():
        print(f"Directory {bins_dir} not found.")
        return

    seen = set()
    print(f"Watching {bins_dir} for .bin files (every 10 s)... Ctrl+C to stop.")

    try:
        while True:
            bin_files = sorted(bins_dir.glob("*.bin"))
            new_files = [f for f in bin_files if f not in seen]

            for f in new_files:
                print(f"\nLoading {f.name} ...")
                try:
                    pc = load_kitti_bin(f)
                    
                    # Create visualization geometries with coordinate frame and grid
                    geometries = create_visualization_geometries(pc)
                    
                    # Try GUI visualization first, fallback to headless if it fails
                    try:
                        o3d.visualization.draw_geometries(
                            geometries,
                            window_name=f"Viewing {f.name} (with coordinate frame)",
                            width=960,
                            height=720,
                        )
                    except Exception as gui_error:
                        print(f"GUI visualization failed: {gui_error}")
                        print("Falling back to headless rendering...")
                        
                        # Headless rendering - save as image
                        vis = o3d.visualization.Visualizer()
                        vis.create_window(visible=False, width=960, height=720)
                        
                        # Add all geometries
                        for geom in geometries:
                            vis.add_geometry(geom)
                        
                        vis.update_geometry(geometries[0])  # Update point cloud
                        vis.poll_events()
                        vis.update_renderer()
                        
                        # Save screenshot
                        screenshot_path = f"{f.stem}_with_frame_screenshot.png"
                        vis.capture_screen_image(screenshot_path)
                        print(f"Screenshot saved as: {screenshot_path}")
                        
                        vis.destroy_window()
                    
                    seen.add(f)
                except Exception as e:
                    print(f"Error loading {f.name}: {e}")

            time.sleep(10.0)
    except KeyboardInterrupt:
        print("\nStopped viewer.")


if __name__ == "__main__":
    main()
