#!/usr/bin/env python3
import argparse
from pathlib import Path
import numpy as np
import open3d as o3d

def load_kitti_bin(path: Path) -> o3d.geometry.PointCloud:
    data = np.fromfile(path, dtype=np.float32)
    if data.size % 4 != 0:
        raise ValueError(f"{path} length={data.size} not divisible by 4 (expected x,y,z,int).")
    pts = data.reshape(-1, 4)
    xyz = pts[:, :3]
    inten = pts[:, 3]

    # Normalize intensity to [0,1] for grayscale coloring
    denom = np.ptp(inten)
    norm_i = (inten - inten.min()) / (denom + 1e-8)
    colors = np.stack([norm_i, norm_i, norm_i], axis=1)

    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(xyz.astype(np.float64, copy=False))
    pc.colors = o3d.utility.Vector3dVector(colors.astype(np.float64, copy=False))
    return pc

def main():

    bin_path = Path("/workspace/navpilot_ws/src/pc2_to_bin/bins/000027.bin")
    voxel = 0.0

    pc = load_kitti_bin(bin_path)
    if voxel > 0:
        pc = pc.voxel_down_sample(voxel)

    o3d.visualization.draw_geometries(
        [pc],
        window_name=f"Open3D — {bin_path.name}",
        width=960,
        height=720,
    )

if __name__ == "__main__":
    main()
