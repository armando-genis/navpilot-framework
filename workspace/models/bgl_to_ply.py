#!/usr/bin/env python3
import numpy as np
import trimesh
from PIL import Image
from plyfile import PlyData, PlyElement

def bake_texture_to_vertex_colors(input_mesh_path, texture_image_path, output_mesh_path):
    # Load mesh and original PLY data
    mesh = trimesh.load(input_mesh_path, process=False)
    orig_ply = PlyData.read(input_mesh_path)
    face_data = orig_ply['face'].data

    # Extract UVs (try Trimesh visual then PLY s,t)
    uv = None
    vis = mesh.visual
    if hasattr(vis, 'uv') and vis.uv is not None:
        uv_data = vis.uv
        if hasattr(vis, 'uv_index') and vis.uv_index is not None:
            uv = uv_data[vis.uv_index].reshape(-1, 3, 2)
        elif uv_data.ndim == 2 and uv_data.shape[0] == mesh.faces.shape[0] * 3:
            uv = uv_data.reshape(-1, 3, 2)
        elif uv_data.ndim == 2 and uv_data.shape[0] == mesh.vertices.shape[0]:
            uv = uv_data[mesh.faces]
    if uv is None:
        vert = orig_ply['vertex'].data
        if 's' in vert.dtype.names and 't' in vert.dtype.names:
            s = np.array(vert['s'], dtype=np.float64)
            t = np.array(vert['t'], dtype=np.float64)
            uv = np.column_stack((s, t))[mesh.faces]
        else:
            raise ValueError("UV coordinates not found in mesh.visual or PLY 's','t' fields.")

    # Load texture
    img = Image.open(texture_image_path).convert('RGB')
    w, h = img.size

    # Sample face-vertex colors (no V-flip)
    fvc = np.zeros((mesh.faces.shape[0], 3, 3), dtype=np.uint8)
    for fi, face in enumerate(mesh.faces):
        for vi in range(3):
            u, v = uv[fi, vi]
            # sample without flipping v-axis
            x = min(max(int(u * w), 0), w - 1)
            y = min(max(int(v * h), 0), h - 1)
            fvc[fi, vi] = img.getpixel((x, y))

    # Average per-vertex colors
    n = mesh.vertices.shape[0]
    vc_sum = np.zeros((n, 3), dtype=np.float64)
    counts = np.zeros(n, dtype=int)
    for fi, face in enumerate(mesh.faces):
        for vi, vid in enumerate(face):
            vc_sum[vid] += fvc[fi, vi]
            counts[vid] += 1
    counts[counts == 0] = 1
    vc = (vc_sum / counts[:, None]).astype(np.uint8)

    # Prepare normals from mesh
    normals = mesh.vertex_normals.astype(np.float32)

    # Build vertex array: x, y, z, nx, ny, nz, red, green, blue
    vertex_array = np.empty(n, dtype=[
        ('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
        ('nx', 'f4'), ('ny', 'f4'), ('nz', 'f4'),
        ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')
    ])
    verts = mesh.vertices.astype(np.float32)
    vertex_array['x'] = verts[:, 0]
    vertex_array['y'] = verts[:, 1]
    vertex_array['z'] = verts[:, 2]
    vertex_array['nx'] = normals[:, 0]
    vertex_array['ny'] = normals[:, 1]
    vertex_array['nz'] = normals[:, 2]
    vertex_array['red']   = vc[:, 0]
    vertex_array['green'] = vc[:, 1]
    vertex_array['blue']  = vc[:, 2]

    # Describe PLY elements
    vert_elem = PlyElement.describe(vertex_array, 'vertex')
    face_elem = PlyElement.describe(face_data, 'face')

    # Write binary little endian PLY
    PlyData([vert_elem, face_elem], text=False).write(output_mesh_path)
    print(f"Saved colored+normals PLY to {output_mesh_path}")

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(
        description='Bake PNG texture into PLY with vertex colors & normals (binary little endian)'
    )
    parser.add_argument('mesh', help='Input PLY mesh path')
    parser.add_argument('texture', help='PNG texture path')
    parser.add_argument('output', help='Output PLY path')
    args = parser.parse_args()
    bake_texture_to_vertex_colors(args.mesh, args.texture, args.output)
