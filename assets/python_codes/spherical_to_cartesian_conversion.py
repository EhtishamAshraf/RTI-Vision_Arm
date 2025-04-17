"""
3D Visualization of Spherical Coordinates

This script visualizes a set of waypoints distributed on the surface of a sphere 
using spherical coordinates (radius, phi, and theta). The plot includes the following:

- A sphere with actual waypoints (as in ROS) plotted in 3D space.
- A selected waypoint, with its corresponding vector (ρ) drawn from the object center.
- Projection of the vector onto the XY plane for better understanding of the spherical angles.
- The arcs representing the azimuthal angle (θ) and elevation angle (ϕ), showing the relationship between the center and the selected waypoint.

For reference: 
Check the function: "get_spherical_waypoints_and_directions" in the file: "utilities.py"
"""

import matplotlib.pyplot as plt
import numpy as np
import math
from mpl_toolkits.mplot3d import Axes3D

# Parameters
center_x, center_y, fixed_z = -0.803, 0.716, 1.037  # Object center
radius = 0.28
num_phi = 6
num_theta = 5

# Store points
x_points = []
y_points = []
z_points = []

# Pick one waypoint to visualize angles
phi_idx_show = 3
theta_idx_show = 4

# Generate spherical coordinates
for phi_idx in range(num_phi):
    phi = math.radians(65 - (phi_idx * 65 / num_phi))
    for theta_idx in range(num_theta):
        theta = math.radians(-60 + (theta_idx * 95 / (num_theta - 1)))

        x = center_x + radius * math.cos(phi) * math.cos(theta)
        y = center_y + radius * math.cos(phi) * math.sin(theta)
        z = fixed_z + radius * math.sin(phi)

        x_points.append(x)
        y_points.append(y)
        z_points.append(z)

        if phi_idx == phi_idx_show and theta_idx == theta_idx_show:
            phi_vis = phi
            theta_vis = theta
            x_vis, y_vis, z_vis = x, y, z

# Plot setup with smaller figure size
fig = plt.figure(figsize=(8, 6))  # Smaller size for better fitting
ax = fig.add_subplot(111, projection='3d')

# Plot waypoints and object center
ax.scatter(x_points, y_points, z_points, c='b', label='Waypoints', s=10)
ax.scatter(center_x, center_y, fixed_z, c='r', marker='^', label='Object Center', s=50)

# Vector to selected waypoint
ax.plot([center_x, x_vis], [center_y, y_vis], [fixed_z, z_vis], color='purple', linewidth=1.5, label='Vector (ρ)')

# --- Projection ---
x_proj = center_x + radius * math.cos(phi_vis) * math.cos(theta_vis)
y_proj = center_y + radius * math.cos(phi_vis) * math.sin(theta_vis)
z_proj = fixed_z

# Draw projection on XY plane
ax.plot([center_x, x_proj], [center_y, y_proj], [fixed_z, fixed_z], 'gray', linestyle='--', linewidth=1)
ax.plot([x_proj, x_vis], [y_proj, y_vis], [z_proj, z_vis], 'gray', linestyle='--', linewidth=1)

# --- θ arc (in XY plane from +X axis) ---
r_xy = radius * math.cos(phi_vis)
arc_theta = np.linspace(0, theta_vis, 100)
x_theta_arc = center_x + r_xy * np.cos(arc_theta)
y_theta_arc = center_y + r_xy * np.sin(arc_theta)
z_theta_arc = np.full_like(x_theta_arc, fixed_z)
ax.plot(x_theta_arc, y_theta_arc, z_theta_arc, color='orange', linestyle='--', linewidth=1, label=r'$\theta$ (azimuth)')
ax.text(center_x + 0.05, center_y, fixed_z - 0.01, "θ", fontsize=10, color='orange')

# --- ϕ arc (elevation from XY plane up to vector) ---
arc_phi = np.linspace(0, phi_vis, 100)
x_phi_arc = center_x + radius * np.cos(arc_phi) * np.cos(theta_vis)
y_phi_arc = center_y + radius * np.cos(arc_phi) * np.sin(theta_vis)
z_phi_arc = fixed_z + radius * np.sin(arc_phi)
ax.plot(x_phi_arc, y_phi_arc, z_phi_arc, color='purple', linestyle='--', linewidth=1, label=r'$\phi$ (elevation)')
ax.text(x_phi_arc[-1] + 0.02, y_phi_arc[-1], z_phi_arc[-1] + 0.02, "ϕ", fontsize=10, color='purple')

# --- Adding arrows for vector and its projection ---
# Vector as an arrow (ρ)
ax.quiver(center_x, center_y, fixed_z, x_vis - center_x, y_vis - center_y, z_vis - fixed_z, color='purple', length=0.5, arrow_length_ratio=0.1)

# Projection vector as an arrow (on XY plane)
ax.quiver(center_x, center_y, fixed_z, x_proj - center_x, y_proj - center_y, 0, color='gray', length=0.5, arrow_length_ratio=0.1)
ax.quiver(x_proj, y_proj, fixed_z, x_vis - x_proj, y_vis - y_proj, z_vis - fixed_z, color='gray', length=0.5, arrow_length_ratio=0.1)

# Coordinate axes with shorter vectors
ax.quiver(center_x, center_y, fixed_z, 0.3, 0, 0, color='red')
ax.quiver(center_x, center_y, fixed_z, 0, 0.3, 0, color='green')
ax.quiver(center_x, center_y, fixed_z, 0, 0, 0.3, color='blue')
ax.text(center_x + 0.32, center_y, fixed_z, 'X', color='red', fontsize=10)
ax.text(center_x, center_y + 0.32, fixed_z, 'Y', color='green', fontsize=10)
ax.text(center_x, center_y, fixed_z + 0.32, 'Z', color='blue', fontsize=10)

# Final plot setup with smaller labels and legend
ax.set_title("Spherical Coordinates Visualization", fontsize=12)
ax.set_xlabel("X", fontsize=10)
ax.set_ylabel("Y", fontsize=10)
ax.set_zlabel("Z", fontsize=10)
ax.legend(fontsize=8)
ax.grid(True)
ax.view_init(elev=25, azim=135)
plt.tight_layout()
plt.show()
