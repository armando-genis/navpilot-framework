#!/usr/bin/env python3
import open3d as o3d
import numpy as np
import copy

def load_mesh(path):
    mesh = o3d.io.read_triangle_mesh(path)
    if mesh.is_empty():
        raise RuntimeError(f"Failed to load mesh from '{path}'")
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    return mesh

def apply_transform(mesh, x, y, z, deg_z):
    """Deep-copy mesh, rotate about Z by deg_z, then translate by (x,y,z)."""
    m = copy.deepcopy(mesh)
    theta = np.deg2rad(deg_z)
    # rotation about Z axis
    R = m.get_rotation_matrix_from_xyz((0, 0, theta))
    m.rotate(R, center=(0, 0, 0))
    m.translate((x, y, z))
    return m

def main():
    # load your two source meshes
    stop_mesh  = load_mesh("stop.ply")
    light_mesh = load_mesh("traffic_light.ply")



    # Loaded Lanelet2 map with 31 lanelets.
    # Crosswalk id: 1379
    # Crosswalk polygon point: 12.6119, 339.523, -19.5623
    # ----->left_bound_length: 4.1729
    # ----->num_stripes: 5
    # Crosswalk id: 542
    # Crosswalk polygon point: -11.2489, 83.32, -6.4619
    # ----->left_bound_length: 4.71124
    # ----->num_stripes: 5
    # Crosswalk id: 549
    # Crosswalk polygon point: 7.9104, 76.701, -6.6363
    # ----->left_bound_length: 5.02934
    # ----->num_stripes: 5
    # Crosswalk id: 556
    # Crosswalk polygon point: -67.0811, 188.484, -11.3962
    # ----->left_bound_length: 6.23089
    # ----->num_stripes: 6
    # Crosswalk id: 563
    # Crosswalk polygon point: 10.9474, 234.174, -14.6971
    # ----->left_bound_length: 5.7364
    # ----->num_stripes: 5

    # where to put stop signs: [x, y, z, rotation_deg_about_Z]
    stop_positions = [
        [7.5,  74.0, -6.6363,   90],
        [-11.0,  78.5, -6.4619,  180],
        [3,  -2, 0,   0],
    ]

    # where to put traffic lights: [x, y, z, rotation_deg_about_Z]
    traffic_positions = [
        [13.5,  74.0, -6.6363,   0],
        [-11.0,  83.32, -6.4619,  90],
        # [8,  2, 5,  45],
    ]

    # apply transforms and collect
    all_meshes = []
    for x, y, z, deg in stop_positions:
        all_meshes.append(apply_transform(stop_mesh,  x, y, z, deg))
    for x, y, z, deg in traffic_positions:
        all_meshes.append(apply_transform(light_mesh, x, y, z, deg))

    # merge into one mesh
    combined = o3d.geometry.TriangleMesh()
    for m in all_meshes:
        combined += m
    combined.merge_close_vertices(1e-6)

    # write out
    o3d.io.write_triangle_mesh("map_elements.ply", combined)
    print(f"Wrote 'map_elements.ply' with "
          f"{len(stop_positions)} stop signs + "
          f"{len(traffic_positions)} traffic lights.")

if __name__ == "__main__":
    main()


