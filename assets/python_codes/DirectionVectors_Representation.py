"""
3D Direction Vector Visualization and Alignment

   - Computes the direction vector between the goal point and the object's current position in 3D space.
   - Applies a tilt to the direction vector by rotating it around an axis perpendicular to both the direction vector and the Z-axis (representing "up").
   - The tilted direction vector is aligned with the default Y-axis (the forward direction of the end effector in current application).

For reference: 
Check the function: "compute_pose_towards_object" in the file: "utilities.py"
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from mpl_toolkits.mplot3d import Axes3D

# --- Base Direction Vector ---
x_goal, y_goal, z_goal = [-0.64, 0.49, 1.037]                 # 3D point on the circle/sphere
x_obj, y_obj, z_obj = [-0.803, 0.716, 1.037]                  # object position

direction = np.array([x_obj - x_goal, y_obj - y_goal, z_obj - z_goal]) # direction vector
    
direction = direction / np.linalg.norm(direction)

# --- Tilt Angle ---
tilt_angle_rad = np.radians(-10)

# --- Rotation Axis ---
z_vector = np.array([0, 0, 1])        # Z-axis
axis = np.cross(direction, z_vector)  # cross product of z-axis and the direction vector gives the axis of rotation
if np.linalg.norm(axis) < 1e-6:
    axis = np.array([1, 0, 0])
axis = axis / np.linalg.norm(axis)

# --- Apply Tilt ---
tilt_rotation = R.from_rotvec(tilt_angle_rad * axis)  # this gives the rotation which when applied to the vector, the vector will be tilted
tilted_direction = tilt_rotation.apply(direction)
tilted_direction = tilted_direction / np.linalg.norm(tilted_direction)

# --- Align to Default Y-Axis ---
default_y_axis = np.array([0, 1, 0])        # Y-axis is the forward-axis of the End-Effector, so we need to align it with the tilted direction vector
rotation_, _ = R.align_vectors([tilted_direction], [default_y_axis])
aligned_direction = rotation_.apply(default_y_axis)

# --- Quaternion ---
quaternion = rotation_.as_quat()
quaternion = quaternion / np.linalg.norm(quaternion)
print("Quaternion:", quaternion)

# --- Plot ---
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(0, 0, 0, color='black', s=20)

# Original direction
ax.quiver(0, 0, 0, *direction, color='blue', length=0.3, arrow_length_ratio=0.1, label='Original Direction')
ax.text(*(direction * 0.3), 'Direction Vector', color='blue', fontsize=9)

# Tilted direction
ax.quiver(0, 0, 0, *tilted_direction, color='red', length=0.3, arrow_length_ratio=0.1, label='Tilted Direction')
ax.text(*(tilted_direction*0.3), 'Tilted Direction Vector', color='red', fontsize=9)

# Aligned direction
ax.quiver(0, 0, 0, *aligned_direction, color='magenta', length=0.3, arrow_length_ratio=0.1, label='Aligned Direction')
ax.text(*(tilted_direction * 0.5), 'Aligned_Y-axis & Tilted Direction Vector', color='magenta', fontsize=9)

# Default Y-axis
ax.quiver(0, 0, 0, *default_y_axis, color='orange', length=0.3, arrow_length_ratio=0.1, label='Default Y-Axis')
ax.text(*(default_y_axis * 0.35), 'Y Axis', color='orange', fontsize=9)

# Axis of rotation
ax.quiver(0, 0, 0, *axis, color='green', length=0.3, arrow_length_ratio=0.1, label='Rotation Axis')
ax.text(*(axis * 0.35), 'Axis of Rotation', color='green', fontsize=9)

# Z vector
ax.quiver(0, 0, 0, *z_vector, color='purple', length=0.3, arrow_length_ratio=0.1, label='Z Vector')
ax.text(*(z_vector * 0.35), '[0,0,1]', color='purple', fontsize=9)

# --- Add the rotation symbol (↻) on the rotation axis ---
symbol_offset = 0.1
ax.text(axis[0] * symbol_offset, axis[1] * symbol_offset, axis[2] * -0.5, '↻', color='green', fontsize=16, ha='center')

# --- Dotted line from Y-axis to aligned vector ---
start_point = default_y_axis * 0.3  # endpoint of Y-axis arrow
end_point = aligned_direction * 0.3  # endpoint of aligned direction arrow

ax.plot([start_point[0], end_point[0]],
        [start_point[1], end_point[1]],
        [start_point[2], end_point[2]],
        color='black', linestyle='dotted', linewidth=2.5, label='Alignment Difference')


# --- Formatting ---
ax.set_xlim([-0.5, 0.5])
ax.set_ylim([-0.5, 0.5])
ax.set_zlim([-0.5, 0.5])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Visualizing tilted Direction Vector | Aligning EE with the direction vector')
ax.legend()
ax.view_init(elev=25, azim=135)
plt.tight_layout()
plt.show()
