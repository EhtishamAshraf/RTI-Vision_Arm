"""
Visualization of Semi-Circular Waypoints in Polar Coordinates and conversion to cartesian coordinates: 

This script visualizes a semicircle of waypoints around an object,
representing light source directions for the LightBot.
Each waypoint is calculated using polar coordinates with a fixed radius
and varying angles.

Key Features:
- Generates 12 directional waypoints on a semicircle from -70° to ~110°.
- Adjusts unreachable angles to -35° or 55° for robotic constraints.
- Displays angle labels and a visual representation of θ using a curved arc.

For reference: 
Check the function: "get_circular_waypoints_and_directions" in the file: "utilities.py"
"""

import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches
import math

# Object center and radius
center_x, center_y = -0.803, 0.716
radius = 0.28
num_points = 12
draw_vector_angle = -55  # adjust the angle to generate the light vector for visualization

# Semicircle arc
arc_angles_deg = np.linspace(-70, 110, 300) # generating the arc
arc_angles_rad = np.radians(arc_angles_deg)
x_arc = center_x + radius * np.cos(arc_angles_rad)
y_arc = center_y + radius * np.sin(arc_angles_rad)

# Plot setup
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_title(f"Light Direction Vector at θ = {draw_vector_angle}°")

# Draw semicircle arc
ax.plot(x_arc, y_arc, linestyle='--', color='gray', label="Semicircle Arc")

# Waypoint plotting loop
for i in range(num_points):
    original_angle_rad = (math.pi * i / num_points) + math.radians(-70)
    angle_deg = math.degrees(original_angle_rad)

    # Apply angle adjustment
    if -35 < angle_deg < -17:
        angle_deg = -35
    elif 50 < angle_deg < 70:
        angle_deg = 55

    adjusted_angle_rad = math.radians(angle_deg)
    x = center_x + radius * math.cos(adjusted_angle_rad)
    y = center_y + radius * math.sin(adjusted_angle_rad)

    # Draw waypoint
    ax.plot(x, y, 'ko', markersize=4)

    # Custom label position for specific angles
    if round(angle_deg, 1) == -70.0:
        ax.text(x - 0.03, y - 0.02, f"{round(angle_deg, 1)}°", fontsize=10)
    elif round(angle_deg, 1) == -55.0:
        ax.text(x - 0.01, y - 0.025, f"{round(angle_deg, 1)}°", fontsize=10)
    elif round(angle_deg, 1) == -40.0:
        ax.text(x + 0.01, y - 0.02, f"{round(angle_deg, 1)}°", fontsize=10)   
    elif round(angle_deg, 1) == -35.0:
        ax.text(x + 0.01, y - 0.01, f"{round(angle_deg, 1)}°", fontsize=10)
    elif round(angle_deg, 1) == 55.0:
        ax.text(x - 0.05, y - 0.02, f"{round(angle_deg, 1)}°", fontsize=10)
    else:
        ax.text(x, y+0.01, f"{round(angle_deg, 1)}°", fontsize=10)

    """
    Draw vector for draw_vector_angle:
    
    Remember: matplotlib’s ax.arrow() does not take the end point of the arrow.
    Instead, it takes:
    -   a start point (x_start, y_start)
    -   a direction and length, given by (dx, dy) — how far to move from the start
    """
    if round(angle_deg) == draw_vector_angle:
        # Vector line
        ax.arrow(center_x, center_y, x - center_x, y - center_y,
                 head_width=0.008, length_includes_head=True, color='blue', label=f"θ = {draw_vector_angle}°")

        # X and Y component lines
        ax.plot([center_x, x], [center_y, center_y], color='red', linestyle=':')
        ax.plot([x, x], [center_y, y], color='green', linestyle=':')

        # Label x and y components with their values
        ax.text((center_x + x) / 2 - 0.015, center_y - 0.015, f"x = {round(x, 2)}", fontsize=9, color='black')
        ax.text(x - 0.015, (center_y + y) / 2, f"y = {round(y, 2)}", fontsize=9, color='black')

# Draw θ arc
theta_arc = patches.Arc((center_x, center_y), 0.16, 0.16, angle=0, theta1=0, theta2=draw_vector_angle,
                        color='purple', linewidth=1.5)
ax.add_patch(theta_arc)

# θ label (placed slightly farther)
theta_label_angle = math.radians(draw_vector_angle/2)
theta_label_radius = 0.10
theta_label_x = center_x + theta_label_radius * math.cos(theta_label_angle)
theta_label_y = center_y + theta_label_radius * math.sin(theta_label_angle)
ax.text(theta_label_x, theta_label_y, "θ", fontsize=15, color='purple')

# Draw object center
ax.plot(center_x, center_y, 'ko')
ax.text(center_x - 0.03, center_y - 0.03, "Object", fontsize=9)

# Final plot settings
ax.set_xlim(center_x - radius - 0.1, center_x + radius + 0.1)
ax.set_ylim(center_y - radius - 0.1, center_y + radius + 0.1)
ax.grid(True)
ax.legend()
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.show()
