#!/usr/bin/env python3
import sys, os
import trimesh
import numpy as np
import shutil
import re

def place_objects(obj_path, positions, output_path, map_obj=None, map_pos=None):
    """
    Reads an OBJ file, places instances at the specified positions,
    combines with a map model, and creates a combined output OBJ file
    while preserving all materials using direct OBJ file manipulation.
    
    Args:
        obj_path (str): Path to the input OBJ file
        positions (list): List of [x, y, z] positions
        output_path (str): Path to save the combined OBJ
        map_obj (str): Path to the map OBJ file
        map_pos (list): Position [x, y, z] for the map
    """
    try:
        # Create output directory if it doesn't exist
        output_dir = os.path.dirname(output_path)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)
        
        # We'll directly manipulate OBJ files instead of using trimesh for concatenation
        # First, we need to read the stop sign OBJ file to understand its structure
        print(f"Reading stop sign OBJ from {obj_path}")
        with open(obj_path, 'r') as f:
            stop_obj_content = f.read()
        
        # Find the MTL file reference
        mtl_match = re.search(r'mtllib\s+(.+)', stop_obj_content)
        if mtl_match:
            mtl_filename = mtl_match.group(1)
            mtl_path = os.path.join(os.path.dirname(obj_path), mtl_filename)
            print(f"Found MTL reference: {mtl_filename}")
            
            # Get the base name for the output MTL file (derived from output OBJ file)
            output_base = os.path.splitext(os.path.basename(output_path))[0]
            new_mtl_filename = f"{output_base}.mtl"
            output_mtl = os.path.join(output_dir, new_mtl_filename)
            
            # Only copy if source and destination are different
            if os.path.abspath(mtl_path) != os.path.abspath(output_mtl):
                if os.path.exists(mtl_path):
                    print(f"Copying MTL file from {mtl_path} to {output_mtl}")
                    shutil.copy2(mtl_path, output_mtl)
                    
                    # Check for texture references in the MTL and copy them
                    with open(mtl_path, 'r') as mtl_file:
                        mtl_content = mtl_file.read()
                        
                    texture_matches = re.findall(r'map_[^\s]+\s+(.+)', mtl_content)
                    for texture_filename in texture_matches:
                        texture_path = os.path.join(os.path.dirname(mtl_path), texture_filename.strip())
                        output_texture = os.path.join(output_dir, os.path.basename(texture_filename.strip()))
                        
                        if os.path.exists(texture_path) and os.path.abspath(texture_path) != os.path.abspath(output_texture):
                            print(f"Copying texture {texture_filename} to {output_texture}")
                            shutil.copy2(texture_path, output_texture)
                else:
                    print(f"Warning: MTL file {mtl_path} not found")
            else:
                print(f"Skipping MTL copy as source and destination are the same: {mtl_path}")
                # Still need to copy any textures
                if os.path.exists(mtl_path):
                    with open(mtl_path, 'r') as mtl_file:
                        mtl_content = mtl_file.read()
                    
                    texture_matches = re.findall(r'map_[^\s]+\s+(.+)', mtl_content)
                    for texture_filename in texture_matches:
                        texture_path = os.path.join(os.path.dirname(mtl_path), texture_filename.strip())
                        output_texture = os.path.join(output_dir, os.path.basename(texture_filename.strip()))
                        
                        if os.path.exists(texture_path) and os.path.abspath(texture_path) != os.path.abspath(output_texture):
                            print(f"Copying texture {texture_filename} to {output_texture}")
                            shutil.copy2(texture_path, output_texture)
        else:
            print("Warning: No MTL reference found in stop sign OBJ")
            new_mtl_filename = None
        
        # Now let's create the combined OBJ file manually
        with open(output_path, 'w') as out_file:
            # Add MTL reference at the top if found
            if mtl_match:
                # Use the new MTL filename that matches our output file
                out_file.write(f"mtllib {new_mtl_filename}\n")
                
            # Keep track of vertex counts to adjust indices for each instance
            total_v = 0
            total_vt = 0
            total_vn = 0
            
            # Process the map first if provided
            if map_obj is not None and map_pos is not None:
                print(f"Reading map OBJ from {map_obj}")
                with open(map_obj, 'r') as f:
                    map_lines = f.readlines()
                
                # Write a comment to identify the map section
                out_file.write("# Map mesh\n")
                
                # Extract vertices, normals, texture coords, and faces
                map_vertices = []
                map_texcoords = []
                map_normals = []
                map_faces = []
                
                for line in map_lines:
                    if line.startswith('v '):
                        parts = line.strip().split()
                        # Apply translation to vertex coordinates
                        if len(parts) >= 4:  # v x y z [w]
                            x = float(parts[1]) + map_pos[0]
                            y = float(parts[2]) + map_pos[1]
                            z = float(parts[3]) + map_pos[2]
                            map_vertices.append(f"v {x} {y} {z}")
                    elif line.startswith('vt '):
                        map_texcoords.append(line.strip())
                    elif line.startswith('vn '):
                        map_normals.append(line.strip())
                    elif line.startswith('f '):
                        map_faces.append(line.strip())
                
                # Write vertices, texture coords, and normals
                for v in map_vertices:
                    out_file.write(f"{v}\n")
                for vt in map_texcoords:
                    out_file.write(f"{vt}\n")
                for vn in map_normals:
                    out_file.write(f"{vn}\n")
                
                # Write faces
                out_file.write("# Map faces\n")
                for f in map_faces:
                    out_file.write(f"{f}\n")
                
                # Update total counts
                total_v += len(map_vertices)
                total_vt += len(map_texcoords)
                total_vn += len(map_normals)
            
            # Parse the stop sign OBJ to extract vertices, normals, texture coords, and faces
            stop_lines = stop_obj_content.splitlines()
            stop_vertices = []
            stop_texcoords = []
            stop_normals = []
            stop_faces = []
            stop_groups = []
            stop_materials = []
            
            current_group = None
            current_material = None
            
            for line in stop_lines:
                if line.startswith('v '):
                    stop_vertices.append(line.strip())
                elif line.startswith('vt '):
                    stop_texcoords.append(line.strip())
                elif line.startswith('vn '):
                    stop_normals.append(line.strip())
                elif line.startswith('g '):
                    current_group = line.strip()
                    stop_groups.append(current_group)
                elif line.startswith('usemtl '):
                    current_material = line.strip()
                    stop_materials.append(current_material)
                elif line.startswith('f '):
                    # Store the face line along with its group and material
                    stop_faces.append((line.strip(), current_group, current_material))
            
            # Now place each stop sign instance
            for i, pos in enumerate(positions):
                out_file.write(f"\n# Stop sign instance {i+1} at position {pos}\n")
                
                # Write translated vertices
                for v_line in stop_vertices:
                    parts = v_line.split()
                    if len(parts) >= 4:  # v x y z [w]
                        x = float(parts[1]) + pos[0]
                        y = float(parts[2]) + pos[1]
                        z = float(parts[3]) + pos[2]
                        out_file.write(f"v {x} {y} {z}\n")
                
                # Write texture coordinates and normals
                for vt in stop_texcoords:
                    out_file.write(f"{vt}\n")
                for vn in stop_normals:
                    out_file.write(f"{vn}\n")
                
                # Write faces with adjusted indices
                current_group = None
                current_material = None
                
                for face, group, material in stop_faces:
                    # If the group or material changes, write the new directive
                    if group != current_group and group is not None:
                        out_file.write(f"{group}\n")
                        current_group = group
                    
                    if material != current_material and material is not None:
                        out_file.write(f"{material}\n")
                        current_material = material
                    
                    # Adjust face indices
                    face_parts = face.split()
                    adjusted_face = "f"
                    
                    for part in face_parts[1:]:  # Skip the 'f' prefix
                        indices = part.split('/')
                        
                        # Adjust vertex index
                        if indices[0]:
                            v_idx = int(indices[0]) + total_v
                            adjusted_face += f" {v_idx}"
                        
                        # Adjust texture coordinate index
                        if len(indices) > 1:
                            adjusted_face += "/"
                            if indices[1]:
                                vt_idx = int(indices[1]) + total_vt
                                adjusted_face += f"{vt_idx}"
                        
                        # Adjust normal index
                        if len(indices) > 2:
                            adjusted_face += "/"
                            if indices[2]:
                                vn_idx = int(indices[2]) + total_vn
                                adjusted_face += f"{vn_idx}"
                    
                    out_file.write(f"{adjusted_face}\n")
                
                # Update total counts for the next instance
                total_v += len(stop_vertices)
                total_vt += len(stop_texcoords)
                total_vn += len(stop_normals)
            
            print(f"Successfully created combined OBJ with material assignments at {output_path}")
            
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    # Path to the input OBJ file
    input_obj = "models/stop.obj"
    map_obj = "models/tec_map.obj"
    
    # List of positions [x, y, z] for stop signs
    positions = [
        [0, 0, 0],      
        [5, 0, 0],  
        [5, 0, 5],      
    ]
    
    # Position for the map
    map_pos = [0, 0, 0]
    
    # Path to the output OBJ file
    output_obj = "models/stop_map_combined.obj"
    
    # Create the combined mesh with preserved materials
    place_objects(input_obj, positions, output_obj, map_obj=map_obj, map_pos=map_pos)