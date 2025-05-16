#!/usr/bin/env python3
import sys, os
import trimesh
import open3d as o3d
import numpy as np

def place_objects(obj_path, positions, output_path):
    """
    Reads an OBJ file, places instances at the specified positions,
    and creates a combined output OBJ file.
    
    Args:
        obj_path (str): Path to the input OBJ file
        positions (list): List of [x, y, z] positions
        output_path (str): Path to save the combined OBJ
    """
    # Load the original mesh
    original_mesh = trimesh.load_mesh(obj_path)
    
    # Create a list to store all meshes
    meshes = []
    
    # For each position, create a transformed copy of the original mesh
    for pos in positions:
        # Create a copy of the original mesh
        mesh_copy = original_mesh.copy()
        
        # Create a translation matrix
        translation = np.eye(4)
        translation[:3, 3] = pos
        
        # Apply the transformation
        mesh_copy.apply_transform(translation)
        
        # Add to the list of meshes
        meshes.append(mesh_copy)
    
    # Combine all meshes into one
    combined_mesh = trimesh.util.concatenate(meshes)
    
    # Export the combined mesh as OBJ
    combined_mesh.export(output_path)
    
    print(f"Created combined mesh with {len(positions)} instances at {output_path}")
    


if __name__ == "__main__":
    # Path to the input OBJ file
    input_obj = "models/stop.obj"
    

    # List of positions [x, y, z] for Blender (y is up)
    positions = [
        [0, 0, 0],      
        [5, 0, 0],  
        [5, 0, 5],      
    ]
    
    # Path to the output OBJ file
    output_obj = "models/stop_list.obj"
    
    # Create the combined mesh
    place_objects(input_obj, positions, output_obj)