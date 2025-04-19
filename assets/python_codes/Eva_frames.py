import cv2
import numpy as np

img = cv2.imread('/home/esirem/Desktop/Eva.jpeg')  

# Resize the image
scale_percent = 30
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)

img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)

# Frame origins in resized image
frame_origins = {
    '0': (122, 404),
    '1': (122, 352),
    '2': (102, 320),
    '3': (102, 254),
    '4': (122, 215),
    '5': (122, 150),
    '6': (148, 113),
}

frame_axes = {
    '0': {'x': (1, 0),  'z': (0, -1), 'z_type': 'arrow'},   
    '1': {'x': (-1, 0),  'z': (0, -1), 'z_type': 'arrow'},
    '2': {'x': (0, -1),  'z': (0, 0),  'z_type': 'dot'},     
    '3': {'x': (1, 0),  'z': (0, 0),  'z_type': 'dot'},     
    '4': {'x': (1, 0),  'z': (0, -1), 'z_type': 'arrow'},
    '5': {'x': (1, 0),  'z': (0, 0),  'z_type': 'dot'},    
    '6': {'x': (1, 0),  'z': (0, -1), 'z_type': 'arrow'},
}

color_x = (0, 0, 255)   # Red
color_y = (0, 255, 0)   # Green
color_z = (255, 0, 0)   # Blue

axis_len = 20   # Length of axis arrows
dot_radius = 5  # Radius for the dots


# Draw frames
for frame, origin in frame_origins.items():
    x, y = origin
    
    dirs = frame_axes[frame]
    x_dir = dirs['x']
    z_dir = dirs['z']
    z_type = dirs['z_type']

    # Draw X axis
    cv2.arrowedLine(img, (x, y), (x + axis_len * x_dir[0], y + axis_len * x_dir[1]), color_x, 2, tipLength=0.3)

    # Draw Z axis
    if z_type == 'arrow':
        cv2.arrowedLine(img, (x, y), (x + axis_len * z_dir[0], y + axis_len * z_dir[1]), color_z, 2, tipLength=0.3)
    elif z_type == 'dot':
        cv2.circle(img, (x + axis_len * z_dir[0], y + axis_len * z_dir[1]), dot_radius, color_z, -1)

cv2.imshow('EVA Arm with Frames', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

cv2.imwrite('/home/esirem/Desktop/eva_with_frames.png', img)

