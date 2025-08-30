#!/usr/bin/env python3
import numpy as np
import trimesh
from PIL import Image
from plyfile import PlyElement, PlyData

def bake_glb_to_ply_vertex_colors(glb_path, texture_image_path, output_ply_path):
    # Load GLB; may return Scene or Trimesh
    loaded = trimesh.load(glb_path, process=False)
    mesh = trimesh.util.concatenate(tuple(loaded.geometry.values())) if isinstance(loaded, trimesh.Scene) else loaded

    if mesh.is_empty:
        raise ValueError(f"Failed to load mesh from {glb_path}")

    # Ensure UVs exist
    vis = mesh.visual
    if not getattr(vis, 'uv', None) is not None:
        raise ValueError("GLB mesh has no UV coordinates")
    uv_data = vis.uv

    # Determine per-face-vertex UVs in correct order
    if getattr(vis, 'uv_index', None) is not None:
        uv = uv_data[vis.uv_index].reshape(-1, 3, 2)
    elif uv_data.shape[0] == mesh.vertices.shape[0]:
        uv = uv_data[mesh.faces]
    elif uv_data.shape[0] == mesh.faces.shape[0] * 3:
        uv = uv_data.reshape(-1, 3, 2)
    else:
        raise ValueError(f"Unexpected UV data length: {uv_data.shape[0]}")

    # Load texture image
    img = Image.open(texture_image_path).convert('RGB')
    w, h = img.size

    # Sample per-face-vertex texture with V-axis flip
    fvc = np.zeros((mesh.faces.shape[0], 3, 3), dtype=np.uint8)
    for fi, face in enumerate(mesh.faces):
        for vi in range(3):
            u, v = uv[fi, vi]
            x = np.clip(int(u * (w - 1)), 0, w - 1)
            y = np.clip(int((1.0 - v) * (h - 1)), 0, h - 1)
            fvc[fi, vi] = img.getpixel((x, y))

    # Compute per-vertex colors by averaging incident face-vertex colors
    n_verts = mesh.vertices.shape[0]
    vc_accum = np.zeros((n_verts, 3), dtype=np.float64)
    counts = np.zeros(n_verts, dtype=int)
    for fi, face in enumerate(mesh.faces):
        for vi, vid in enumerate(face):
            vc_accum[vid] += fvc[fi, vi]
            counts[vid] += 1
    counts[counts == 0] = 1
    vertex_colors = (vc_accum / counts[:, None]).astype(np.uint8)

    # Build vertex array: x, y, z, red, green, blue (preserving GLB orientation)
    positions = mesh.vertices.astype(np.float32)
    ply_dtype = [
        ('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
        ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')
    ]
    verts = np.empty(n_verts, dtype=ply_dtype)
    verts['x'], verts['y'], verts['z'] = positions.T
    verts['red'], verts['green'], verts['blue'] = vertex_colors.T

    # Build face element
    n_faces = mesh.faces.shape[0]
    face_dtype = [('vertex_indices', 'i4', (3,))]
    faces = np.empty(n_faces, dtype=face_dtype)
    faces['vertex_indices'] = mesh.faces

    # Write binary little endian PLY
    PlyData([
        PlyElement.describe(verts, 'vertex'),
        PlyElement.describe(faces, 'face')
    ], text=False).write(output_ply_path)
    print(f"Saved baked PLY (matching GLB orientation) to {output_ply_path}")

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(
        description='Bake texture from GLB UVs into binary_little_endian PLY vertex colors only, preserving orientation'
    )
    parser.add_argument('glb', help='Input GLB path')
    parser.add_argument('texture', help='PNG texture path')
    parser.add_argument('output', help='Output PLY path')
    args = parser.parse_args()
    bake_glb_to_ply_vertex_colors(args.glb, args.texture, args.output)
