#!/usr/bin/env python3
"""
Quick RoboCup SPL Field Map Generator
Creates PGM image + YAML file for AMCL
Run this in ~/basbot directory
"""

import numpy as np
import cv2
import os

print("🎯 Creating RoboCup SPL Field Map...")

# Field dimensions (meters) - RoboCup SPL 2024
FIELD_LENGTH = 9.0
FIELD_WIDTH = 6.0
CENTER_CIRCLE_RADIUS = 0.75
PENALTY_LENGTH = 1.65
PENALTY_WIDTH = 3.9
GOAL_WIDTH = 2.6
LINE_WIDTH = 0.05

# Map resolution (meters per pixel)
RESOLUTION = 0.02  # 2cm per pixel = good balance

# Calculate map size in pixels
MAP_WIDTH_PX = int(FIELD_WIDTH / RESOLUTION)
MAP_HEIGHT_PX = int(FIELD_LENGTH / RESOLUTION)

print(f"📐 Map size: {MAP_WIDTH_PX}x{MAP_HEIGHT_PX} pixels")
print(f"📏 Resolution: {RESOLUTION}m/pixel ({1/RESOLUTION:.0f} pixels/meter)")

# Create blank map (255=free space, 0=occupied/lines)
map_img = np.ones((MAP_HEIGHT_PX, MAP_WIDTH_PX), dtype=np.uint8) * 254

def draw_field_line(img, x1_m, y1_m, x2_m, y2_m, thickness_m):
    """
    Draw line on map in field coordinates
    Field center is (0,0), X=forward, Y=left
    """
    # Convert meters to pixels
    cx = MAP_WIDTH_PX // 2
    cy = MAP_HEIGHT_PX // 2
    
    # Field coords: (0,0) at center, X=forward(up), Y=left
    # Image coords: (0,0) at top-left, X=right, Y=down
    px1 = int(cx + y1_m / RESOLUTION)  # y → horizontal
    py1 = int(cy - x1_m / RESOLUTION)  # x → vertical (inverted)
    px2 = int(cx + y2_m / RESOLUTION)
    py2 = int(cy - x2_m / RESOLUTION)
    
    thickness_px = max(1, int(thickness_m / RESOLUTION))
    cv2.line(img, (px1, py1), (px2, py2), 0, thickness_px)
    return img

# Draw all field lines
line_w = LINE_WIDTH
half_l = FIELD_LENGTH / 2
half_w = FIELD_WIDTH / 2

print("✏️  Drawing field lines...")

# Outer boundary
draw_field_line(map_img, -half_l, -half_w, half_l, -half_w, line_w)  # Right sideline
draw_field_line(map_img, -half_l, half_w, half_l, half_w, line_w)    # Left sideline
draw_field_line(map_img, -half_l, -half_w, -half_l, half_w, line_w)  # Bottom goal line
draw_field_line(map_img, half_l, -half_w, half_l, half_w, line_w)    # Top goal line

# Center line
draw_field_line(map_img, 0, -half_w, 0, half_w, line_w)

# Center circle
cx = MAP_WIDTH_PX // 2
cy = MAP_HEIGHT_PX // 2
radius_px = int(CENTER_CIRCLE_RADIUS / RESOLUTION)
cv2.circle(map_img, (cx, cy), radius_px, 0, max(1, int(line_w / RESOLUTION)))

# Penalty areas - Top (positive X)
pen_x = half_l - PENALTY_LENGTH
pen_y_half = PENALTY_WIDTH / 2
draw_field_line(map_img, pen_x, -pen_y_half, pen_x, pen_y_half, line_w)  # Penalty box line
draw_field_line(map_img, half_l, -pen_y_half, pen_x, -pen_y_half, line_w)  # Right side
draw_field_line(map_img, half_l, pen_y_half, pen_x, pen_y_half, line_w)    # Left side

# Penalty areas - Bottom (negative X)
pen_x = -half_l + PENALTY_LENGTH
draw_field_line(map_img, pen_x, -pen_y_half, pen_x, pen_y_half, line_w)
draw_field_line(map_img, -half_l, -pen_y_half, pen_x, -pen_y_half, line_w)
draw_field_line(map_img, -half_l, pen_y_half, pen_x, pen_y_half, line_w)

# Penalty spots (small circles)
pen_spot_radius = max(1, int(0.05 / RESOLUTION))
# Top penalty spot
spot_x = half_l - 1.3  # 1.3m from goal line
cv2.circle(map_img, (cx, int(cy - spot_x / RESOLUTION)), pen_spot_radius, 0, -1)
# Bottom penalty spot
spot_x = -half_l + 1.3
cv2.circle(map_img, (cx, int(cy - spot_x / RESOLUTION)), pen_spot_radius, 0, -1)

print("💾 Saving map files...")

# Create output directory
output_dir = 'src/soccer_object_localization/maps'
os.makedirs(output_dir, exist_ok=True)

# Save PGM file
pgm_path = os.path.join(output_dir, 'soccer_field.pgm')
cv2.imwrite(pgm_path, map_img)
print(f"✅ PGM saved: {pgm_path}")

# Create YAML file
yaml_content = f"""image: soccer_field.pgm
resolution: {RESOLUTION}
origin: [{-FIELD_WIDTH/2:.3f}, {-FIELD_LENGTH/2:.3f}, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
"""

yaml_path = os.path.join(output_dir, 'soccer_field.yaml')
with open(yaml_path, 'w') as f:
    f.write(yaml_content)
print(f"✅ YAML saved: {yaml_path}")

# Display map preview
print("\n📊 Map Statistics:")
print(f"  Total pixels: {MAP_WIDTH_PX * MAP_HEIGHT_PX:,}")
print(f"  Map dimensions: {MAP_WIDTH_PX}x{MAP_HEIGHT_PX} px")
print(f"  Field dimensions: {FIELD_WIDTH}x{FIELD_LENGTH} m")
print(f"  Occupied pixels (lines): {np.sum(map_img < 128):,}")
print(f"  Free pixels: {np.sum(map_img >= 128):,}")

# Show map
print("\n🖼️  Displaying map preview (close window to continue)...")
display_img = cv2.resize(map_img, (800, 1200))  # Scale for display
cv2.imshow('Soccer Field Map', display_img)
cv2.waitKey(2000)  # Show for 2 seconds
cv2.destroyAllWindows()

print("\n✨ Map creation complete!")
print(f"\n📂 Files created:")
print(f"  - {pgm_path}")
print(f"  - {yaml_path}")
print("\n🔧 Next steps:")
print("  1. cd ~/basbot")
print("  2. colcon build --packages-select soccer_object_localization")
print("  3. source install/setup.bash")
print("  4. Relaunch: ros2 launch soccer_object_localization amcl_no_odom.launch.py")