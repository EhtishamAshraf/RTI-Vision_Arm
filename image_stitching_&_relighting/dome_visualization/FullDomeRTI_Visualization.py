"""
3D Visualization of rotation applid to second acquisition of light positions for full-dome RTI - just for visualization

It does the following:

- It assumes that the object was rotated between the two acquisition because of a limitation of the Robotic Arm.
- It takes the rotaton of the object and gets the rotation matrix.
- It takes the inverse of the rotation matrix to generate light vectors for the other half of the dome for second acquisition.

For reference: 
Check the code: "spherical_to_cartesian_conversion.py" in the folder in the ros workspace of RTI Bot: "assets/python_codes"
"""

import matplotlib.pyplot as plt
import numpy as np
import math
from mpl_toolkits.mplot3d import Axes3D

# Parameters
center_x, center_y, fixed_z = -0.803, 0.716, 1.037  # Object's center
radius = 0.28
num_phi = 6
num_theta = 5

# arrays to store points
x_points = []
y_points = []
z_points = []

x_points_rot = []
y_points_rot = []
z_points_rot = []

# one waypoint to visualize angles
phi_idx_show = 3
theta_idx_show = 4

# Rotation matrix for Acquisition 2 (object rotated 180° about Z-axis)
theta_deg = -180
theta_rad = math.radians(theta_deg)

# Rotation matrix around Z-axis
rotation_matrix_z_inv = np.array([
    [math.cos(theta_rad), -math.sin(theta_rad), 0],
    [math.sin(theta_rad),  math.cos(theta_rad), 0],
    [0,                    0,                    1]
])

# Generate spherical coordinates
for phi_idx in range(num_phi):
    phi = math.radians(65 - (phi_idx * 65 / num_phi))
    for theta_idx in range(num_theta):
        theta = math.radians(-60 + (theta_idx * 95 / (num_theta - 1)))

        # Original light position (Acquisition 1)
        x = center_x + radius * math.cos(phi) * math.cos(theta)
        y = center_y + radius * math.cos(phi) * math.sin(theta)
        z = fixed_z + radius * math.sin(phi)

        x_points.append(x)
        y_points.append(y)
        z_points.append(z)

        # Rotated version of light position (Acquisition 2)
        vec = np.array([x - center_x, y - center_y, z - fixed_z])  # light vector in world
        vec_rotated = rotation_matrix_z_inv @ vec
        x_rot = center_x + vec_rotated[0]
        y_rot = center_y + vec_rotated[1]
        z_rot = fixed_z + vec_rotated[2]

        x_points_rot.append(x_rot)
        y_points_rot.append(y_rot)
        z_points_rot.append(z_rot)

        # for arc visualization
        if phi_idx == phi_idx_show and theta_idx == theta_idx_show:
            phi_vis = phi
            theta_vis = theta
            x_vis, y_vis, z_vis = x, y, z

print("x_points:", x_points)
print("x_points_rot", x_points_rot)

# Plot setup
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

# Plot original and rotated waypoints
ax.scatter(x_points, y_points, z_points, c='blue', label='Original Waypoints', s=10)
ax.scatter(x_points_rot, y_points_rot, z_points_rot, c='green', label='Rotated Waypoints', s=10)

# Object center
ax.scatter(center_x, center_y, fixed_z, c='r', marker='^', label='Object Center', s=50)

# Vector to selected waypoint
ax.plot([center_x, x_vis], [center_y, y_vis], [fixed_z, z_vis], color='purple', linewidth=1.5, label='Vector (ρ)')

# Projection
x_proj = center_x + radius * math.cos(phi_vis) * math.cos(theta_vis)
y_proj = center_y + radius * math.cos(phi_vis) * math.sin(theta_vis)
z_proj = fixed_z

# Draw projection on XY plane
ax.plot([center_x, x_proj], [center_y, y_proj], [fixed_z, fixed_z], 'gray', linestyle='--', linewidth=1)
ax.plot([x_proj, x_vis], [y_proj, y_vis], [z_proj, z_vis], 'gray', linestyle='--', linewidth=1)

# θ arc (azimuth) 
r_xy = radius * math.cos(phi_vis)
arc_theta = np.linspace(0, theta_vis, 100)
x_theta_arc = center_x + r_xy * np.cos(arc_theta)
y_theta_arc = center_y + r_xy * np.sin(arc_theta)
z_theta_arc = np.full_like(x_theta_arc, fixed_z)
ax.plot(x_theta_arc, y_theta_arc, z_theta_arc, color='orange', linestyle='--', linewidth=1, label=r'$\theta$ (azimuth)')
ax.text(center_x + 0.05, center_y, fixed_z - 0.01, "θ", fontsize=10, color='orange')

# ϕ arc (elevation)
arc_phi = np.linspace(0, phi_vis, 100)
x_phi_arc = center_x + radius * np.cos(arc_phi) * np.cos(theta_vis)
y_phi_arc = center_y + radius * np.cos(arc_phi) * np.sin(theta_vis)
z_phi_arc = fixed_z + radius * np.sin(arc_phi)
ax.plot(x_phi_arc, y_phi_arc, z_phi_arc, color='purple', linestyle='--', linewidth=1, label=r'$\phi$ (elevation)')
ax.text(x_phi_arc[-1] + 0.02, y_phi_arc[-1], z_phi_arc[-1] + 0.02, "ϕ", fontsize=10, color='purple')

# Arrows
ax.quiver(center_x, center_y, fixed_z, x_vis - center_x, y_vis - center_y, z_vis - fixed_z, color='purple', length=0.5, arrow_length_ratio=0.1)
ax.quiver(center_x, center_y, fixed_z, x_proj - center_x, y_proj - center_y, 0, color='gray', length=0.5, arrow_length_ratio=0.1)
ax.quiver(x_proj, y_proj, fixed_z, x_vis - x_proj, y_vis - y_proj, z_vis - fixed_z, color='gray', length=0.5, arrow_length_ratio=0.1)

# Coordinate axes
ax.quiver(center_x, center_y, fixed_z, 0.3, 0, 0, color='red')
ax.quiver(center_x, center_y, fixed_z, 0, 0.3, 0, color='green')
ax.quiver(center_x, center_y, fixed_z, 0, 0, 0.3, color='blue')
ax.text(center_x + 0.32, center_y, fixed_z, 'X', color='red', fontsize=10)
ax.text(center_x, center_y + 0.32, fixed_z, 'Y', color='green', fontsize=10)
ax.text(center_x, center_y, fixed_z + 0.32, 'Z', color='blue', fontsize=10)

# Final plot setup
ax.set_title("Full-Dome RTI Light Positions", fontsize=12)
ax.set_xlabel("X", fontsize=10)
ax.set_ylabel("Y", fontsize=10)
ax.set_zlabel("Z", fontsize=10)
ax.legend(fontsize=8)
ax.grid(True)
ax.view_init(elev=25, azim=135)
plt.tight_layout()
plt.show()

