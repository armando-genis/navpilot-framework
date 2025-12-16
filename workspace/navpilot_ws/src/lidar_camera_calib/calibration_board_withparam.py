import cv2
import numpy as np
from PIL import Image
import base64
import io

# ========= PARAMETERS =========
# Board size in meters
board_width_m = 1.2
board_height_m = 0.6   # <- this is your new height

# We'll use 1 mm = 1 pixel
mm_per_m = 1000.0
board_width_px = int(board_width_m * mm_per_m)   # 1200 px
board_height_px = int(board_height_m * mm_per_m) # 600 px

# ArUco parameters
aruco_dict_name = cv2.aruco.DICT_6X6_250
marker_ids = [1, 2, 3, 4]  # clockwise from top-left
marker_edge_m = 0.18
marker_edge_px = int(marker_edge_m * mm_per_m)   # 180 px

offset_corner_m = 0.05
offset_corner_px = int(offset_corner_m * mm_per_m)  # 50 px

# Circular cutouts
circle_radius_m = 0.12
circle_radius_px = int(circle_radius_m * mm_per_m)  # 120 px

offset_center_m = 0.15
offset_center_px = int(offset_center_m * mm_per_m)  # 150 px

# ========= CREATE BOARD =========
# White background
board = np.ones((board_height_px, board_width_px), dtype=np.uint8) * 255

center_x = board_width_px // 2
center_y = board_height_px // 2

# ========= DRAW CIRCULAR CUTOUTS =========
# 3 asymmetric positions around center
circle_centers = [
    (center_x + offset_center_px, center_y + offset_center_px),  # (+0.15, +0.15)  image coords
    (center_x - offset_center_px, center_y + offset_center_px),  # (-0.15, +0.15)
    (center_x, center_y - offset_center_px),                     # (0, -0.15)    image coords
]

for (cx, cy) in circle_centers:
    cv2.circle(board, (cx, cy), circle_radius_px, color=0, thickness=2)  # outline only

# ========= DRAW ARUCO MARKERS =========
aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_name)

def place_aruco(board_img, marker_id, center, size_px):
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, size_px)
    x_center, y_center = center

    x1 = int(x_center - size_px / 2)
    y1 = int(y_center - size_px / 2)
    x2 = x1 + size_px
    y2 = y1 + size_px

    # Safety clipping
    x1 = max(0, x1)
    y1 = max(0, y1)
    x2 = min(board_img.shape[1], x2)
    y2 = min(board_img.shape[0], y2)

    # Place marker
    board_img[y1:y2, x1:x2] = marker_img[0:(y2 - y1), 0:(x2 - x1)]


# Compute marker centers (clockwise from top-left)
# Top-left corner is (0, 0), origin at top-left in image space
centers_aruco = {}

# Top-left (ID 1)
centers_aruco[1] = (
    offset_corner_px + marker_edge_px // 2,
    offset_corner_px + marker_edge_px // 2
)

# Top-right (ID 2)
centers_aruco[2] = (
    board_width_px - offset_corner_px - marker_edge_px // 2,
    offset_corner_px + marker_edge_px // 2
)

# Bottom-right (ID 3)
centers_aruco[3] = (
    board_width_px - offset_corner_px - marker_edge_px // 2,
    board_height_px - offset_corner_px - marker_edge_px // 2
)

# Bottom-left (ID 4)
centers_aruco[4] = (
    offset_corner_px + marker_edge_px // 2,
    board_height_px - offset_corner_px - marker_edge_px // 2
)

# Place all markers
for mid in marker_ids:
    place_aruco(board, mid, centers_aruco[mid], marker_edge_px)

# ========= SAVE RESULT =========
cv2.imwrite("calibration_board.png", board)
print("Saved calibration_board.png")

# ========= SAVE PDF VIA PILLOW =========
img = Image.fromarray(board, mode="L")  # 'L' = 8-bit grayscale
img = img.convert("RGB")  # PDF likes RGB better

dpi = 25.4  # so that ~1 px ≈ 1 mm when printed
img.save("calibration_board.pdf", "PDF", resolution=dpi)
print("Saved calibration_board.pdf")

# ========= SAVE SVG (VECTORIZED) =========
def image_to_base64(img_array):
    """Convert numpy array to base64-encoded PNG string"""
    img_pil = Image.fromarray(img_array, mode="L")
    img_pil = img_pil.convert("RGB")
    buffer = io.BytesIO()
    img_pil.save(buffer, format="PNG")
    img_str = base64.b64encode(buffer.getvalue()).decode()
    return f"data:image/png;base64,{img_str}"

# Generate ArUco markers as base64 images
aruco_images_base64 = {}
for mid in marker_ids:
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, mid, marker_edge_px)
    aruco_images_base64[mid] = image_to_base64(marker_img)

board_width_mm = board_width_px
board_height_mm = board_height_px
svg_content = f'''<?xml version="1.0" encoding="UTF-8"?>
<svg width="{board_width_mm}mm" height="{board_height_mm}mm" 
     viewBox="0 0 {board_width_px} {board_height_px}"
     xmlns="http://www.w3.org/2000/svg"
     xmlns:xlink="http://www.w3.org/1999/xlink">
  <!-- White background -->
  <rect width="{board_width_px}" height="{board_height_px}" fill="white"/>
  
  <!-- Circular cutouts (outlines) -->
'''
for (cx, cy) in circle_centers:
    svg_content += f'  <circle cx="{cx}" cy="{cy}" r="{circle_radius_px}" fill="none" stroke="black" stroke-width="2"/>\n'

svg_content += '\n  <!-- ArUco markers -->\n'
for mid in marker_ids:
    cx, cy = centers_aruco[mid]
    x_pos = cx - marker_edge_px // 2
    y_pos = cy - marker_edge_px // 2
    svg_content += f'''  <image x="{x_pos}" y="{y_pos}" width="{marker_edge_px}" height="{marker_edge_px}" 
         xlink:href="{aruco_images_base64[mid]}"/>\n'''

svg_content += '</svg>'

with open("calibration_board.svg", "w") as f:
    f.write(svg_content)
print("Saved calibration_board.svg (vectorized)")

# ========= SAVE YAML CONFIG =========
# We now compute the board-relative positions used in the YAML

# 1) Marker positions: TOP-LEFT corners in board frame (origin at center, x right, y up)
marker_positions_data = []

for mid in marker_ids:
    cx_px, cy_px = centers_aruco[mid]
    # top-left corner in image coordinates
    x_tl_px = cx_px - marker_edge_px / 2.0
    y_tl_px = cy_px - marker_edge_px / 2.0

    # convert to board-centered coordinates in meters
    # image origin at (0,0) top-left; board origin at center
    x_m = (x_tl_px - board_width_px / 2.0) / mm_per_m
    y_img_m = (y_tl_px - board_height_px / 2.0) / mm_per_m
    y_m = -y_img_m  # flip sign so that y is upwards

    marker_positions_data.extend([x_m, y_m])

# 2) Cutouts: centers + radius, also in board frame
cutouts_data = []
for (cx_px, cy_px) in circle_centers:
    x_m = (cx_px - board_width_px / 2.0) / mm_per_m
    y_img_m = (cy_px - board_height_px / 2.0) / mm_per_m
    y_m = -y_img_m
    cutouts_data.extend([
        1,            # id
        x_m,
        y_m,
        circle_radius_m
    ])

# 3) Build YAML text manually (OpenCV style)
yaml_lines = []
yaml_lines.append("%YAML:1.0")
yaml_lines.append(f"board_width: {board_width_m}")
yaml_lines.append(f"board_height: {board_height_m}")
yaml_lines.append(f"marker_size: {marker_edge_m}")
yaml_lines.append("marker_ids:")
yaml_lines.append(f"    rows: {len(marker_ids)}")
yaml_lines.append("    cols: 1")
yaml_lines.append("    dt: i")
yaml_lines.append("    data: [" + ", ".join(str(int(i)) for i in marker_ids) + "]")

yaml_lines.append("marker_positions:")
yaml_lines.append(f"    rows: {len(marker_ids)}")
yaml_lines.append("    cols: 2")
yaml_lines.append("    dt: f")
yaml_lines.append("    data: [" + ", ".join(f"{v:.8f}" for v in marker_positions_data) + "]")

yaml_lines.append("cutouts:")
yaml_lines.append("    rows: 1")
yaml_lines.append(f"    cols: {len(cutouts_data)}")
yaml_lines.append("    dt: f")
yaml_lines.append("    data: [" + ", ".join(f"{v:.8f}" for v in cutouts_data) + "]")

yaml_lines.append("min_marker_detection: 2")
yaml_lines.append('cad_model_mesh: "calibration_target_3holes_cad_mesh.ply"')
yaml_lines.append('cad_model_cloud: "calibration_target_3holes_cad_cloud.ply"')

yaml_text = "\n".join(yaml_lines)

with open("calibration_board_config.yaml", "w") as f:
    f.write(yaml_text)

print("Saved calibration_board_config.yaml")
