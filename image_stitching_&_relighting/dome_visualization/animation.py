
"""
Code to animate the light direction vectors for RTI (Reflectance Transformation Imaging)
acquisition in a full dome setup.
This code visualizes the light direction vectors for two acquisitions:
1. Original light direction vectors (Acquisition 1)
2. Rotated light direction vectors (Acquisition 2)
The object is assumed to be rotated 180° about the Z-axis between the two acquisitions.
"""
import matplotlib
matplotlib.use('TkAgg')  

import matplotlib.pyplot as plt
import numpy as np
import math
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Parameters
center_x, center_y, fixed_z = -0.803, 0.716, 1.037
radius = 0.28
num_phi = 6
num_theta = 5

# Rotation matrix for -180° Z-axis
theta_deg = -180
theta_rad = math.radians(theta_deg)
rotation_matrix_z_inv = np.array([
    [math.cos(theta_rad), -math.sin(theta_rad), 0],
    [math.sin(theta_rad),  math.cos(theta_rad), 0],
    [0,                   0,                    1]
])

# Precompute light vectors
light_vectors = []
for phi_idx in range(num_phi):
    phi = math.radians(65 - (phi_idx * 65 / num_phi))
    for theta_idx in range(num_theta):
        theta = math.radians(-60 + (theta_idx * 95 / (num_theta - 1)))

        x = center_x + radius * math.cos(phi) * math.cos(theta)
        y = center_y + radius * math.cos(phi) * math.sin(theta)
        z = fixed_z + radius * math.sin(phi)
        vec1 = (x - center_x, y - center_y, z - fixed_z)
        light_vectors.append(('Original', vec1))

        # Rotated version (2nd acquisition)
        vec = np.array(vec1)
        vec_rot = rotation_matrix_z_inv @ vec
        light_vectors.append(('Rotated', tuple(vec_rot)))

# Setup plot
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(center_x, center_y, fixed_z, c='red', marker='^', label='Object Center', s=50)

# Coordinate axes
ax.quiver(center_x, center_y, fixed_z, 0.3, 0, 0, color='red')
ax.quiver(center_x, center_y, fixed_z, 0, 0.3, 0, color='green')
ax.quiver(center_x, center_y, fixed_z, 0, 0, 0.3, color='blue')
ax.text(center_x + 0.32, center_y, fixed_z, 'X', color='red', fontsize=10)
ax.text(center_x, center_y + 0.32, fixed_z, 'Y', color='green', fontsize=10)
ax.text(center_x, center_y, fixed_z + 0.32, 'Z', color='blue', fontsize=10)
ax.set_xlim(center_x - 0.4, center_x + 0.4)
ax.set_ylim(center_y - 0.4, center_y + 0.4)
ax.set_zlim(fixed_z - 0.2, fixed_z + 0.4)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("RTI Light Direction Animation (Full Dome)")

# Persistent arrow handle
vector = ax.quiver(center_x, center_y, fixed_z, 0, 0, 0, color='black')

# Update function
def update(frame):
    global vector  # track it persistently
    label, (vx, vy, vz) = light_vectors[frame]
    vector.remove()  # remove previous vector
    color = 'blue' if label == 'Original' else 'green'
    vector = ax.quiver(center_x, center_y, fixed_z, vx, vy, vz, color=color)
    return vector,

# Animate
ani = FuncAnimation(fig, update, frames=len(light_vectors), interval=300, blit=False, repeat=True)

plt.show()
