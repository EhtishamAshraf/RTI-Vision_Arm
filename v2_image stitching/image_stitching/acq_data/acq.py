"""Acquisition class for handling light positions and images in a structured format."""

import os
import cv2
import numpy as np
import matplotlib.pyplot as plt

class Acq:
    def __init__(self, path, lp_file=None, surface_physical_size=(0.20, 0.12) , resize_factor=1.0):
        """
        Args:
            path (str): Folder containing images and .lp file
            lp_file (str, optional): Path to .lp file with light positions. If None, tries to find in path.
            surface_physical_size (tuple): Physical size of object in meters (width, height)
        """
        self.path = path
        self.lp_file = lp_file or self._find_lp_file()
        self.surface_physical_size = surface_physical_size  # (width, height)
        self.resize_factor = resize_factor

        self.image_paths = []
        self.image_names = []
        self.lps_cartesian = []

        self.image_width = None
        self.image_height = None
        self.image_channels = None

        self.distance_matrices = None
        self.angle_matrices = None

        self._load_images_and_lp()
        self._compute_per_pixel_light_dirs_and_distances()

    def _find_lp_file(self):
        # Find the first .lp file in the directory
        for f in os.listdir(self.path):
            if f.endswith('.lp'):
                return os.path.join(self.path, f)
        raise FileNotFoundError("No .lp file found in folder: " + self.path)

    def _load_images_and_lp(self):
        # Load .lp file light positions and image paths
        with open(self.lp_file, 'r') as f:
            lines = f.readlines()

        # First line should be number of lights
        num_lights = int(lines[0].strip())

        if len(lines[1:]) != num_lights:
            raise ValueError("Number of lights in .lp file does not match header")

        for line in lines[1:]:
            parts = line.strip().split()
            if len(parts) != 4:
                raise ValueError(f"Invalid line in .lp file: {line}")
            img_file, x, y, z = parts
            self.lps_cartesian.append((float(x), float(y), float(z)))

            img_path = os.path.join(self.path, img_file)
            # print("Trying to load image:", img_path)
            if not os.path.exists(img_path):
                raise FileNotFoundError(f"Image {img_file} listed in .lp file not found")
            self.image_paths.append(img_path)
            self.image_names.append(img_file)

        # Read first image to get shape
        img = cv2.imread(self.image_paths[0])
        if img is None:
            raise ValueError(f"Cannot read image: {self.image_paths[0]}")

        # Apply resizing
        if self.resize_factor != 1.0:
            new_size = (int(img.shape[1] * self.resize_factor), int(img.shape[0] * self.resize_factor))
            img = cv2.resize(img, new_size, interpolation=cv2.INTER_AREA)
        
        self.image_height, self.image_width, self.image_channels = img.shape

    def _compute_per_pixel_light_dirs_and_distances(self):
        """
        For each light position, compute distance and angle matrices of shape (H, W)
        relative to the object's physical size.
        """
        w, h = self.image_width, self.image_height
        surface_width, surface_height = self.surface_physical_size

        # Create grid of physical coordinates (x,y) centered at zero
        x_coords = np.linspace(-surface_width / 2, surface_width / 2, w)
        y_coords = np.linspace(-surface_height / 2, surface_height / 2, h)
        X, Y = np.meshgrid(x_coords, y_coords)  # Note: shape (h, w)

        distances = []
        angles = []

        for lp in self.lps_cartesian:
            lp_x, lp_y, lp_z = lp
            # Z is zero because object surface is at z=0
            Z = np.zeros_like(X)

            # Calculate distances from each pixel to light position
            dist = np.sqrt((X - lp_x) ** 2 + (Y - lp_y) ** 2 + (Z - lp_z) ** 2)

            # Calculate angle theta between light direction and surface normal (0,0,1)
            # cos(theta) = -lp_z / distance (assuming surface normal pointing in +Z)
            cos_theta = np.clip(-lp_z / dist, -1.0, 1.0)
            angle = np.arccos(cos_theta)

            distances.append(dist)
            angles.append(angle)

        self.distance_matrices = np.array(distances)  # Shape: (num_lights, H, W)
        self.angle_matrices = np.array(angles)        # Shape: (num_lights, H, W)

        # Optionally save matrices to disk
        np.save(os.path.join(self.path, "distance_matrices.npy"), self.distance_matrices)
        np.save(os.path.join(self.path, "angle_matrices.npy"), self.angle_matrices)
        print("generated .npy file for light correction")

    def plot_light_positions_3d(self, save_path=None):
        """
        Plot 3D scatter of light positions
        """
        from mpl_toolkits.mplot3d import Axes3D
        x, y, z = zip(*self.lps_cartesian)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(x, y, z)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        if save_path:
            plt.savefig(save_path)
        else:
            plt.show()
        plt.close()

    def plot_light_positions_2d(self, save_path=None):
        """
        Plot 2D projection of light positions
        """
        x, y = zip(*[(lp[0], lp[1]) for lp in self.lps_cartesian])
        plt.figure()
        plt.scatter(x, y)
        plt.xlabel("X")
        plt.ylabel("Y")
        if save_path:
            plt.savefig(save_path)
        else:
            plt.show()
        plt.close()
