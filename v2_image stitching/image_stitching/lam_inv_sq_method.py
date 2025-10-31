import cv2
import numpy as np
from acq_data import Acq
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

class LamInvSqMethod:
    def __init__(self, acq, save_path):
        self.save_path = save_path
        self.corrected_image_paths = []
        self.acq = acq
        self.surface_physical_size = [0.236, 0.128]
        self.DISTANCE_CORRECTION_GAIN = 2
        self.LAMBERT_CORRECTION_GAIN = 0.6

        results_folder = os.path.join(self.save_path, "results")
        os.makedirs(results_folder, exist_ok=True)  # creates 'results' if it doesn't exist


        # Process each image
        for i in range(len(acq.image_paths)):
            corrected_image_name = acq.image_names[i]
            self.correct_image(acq.image_paths[i], acq.lps_cartesian[i], corrected_image_name, light_index=i)

        # Create a new Acq object for corrected images
        self.corrected_acq = Acq(results_folder, lp_file=acq.lp_file)

    def correct_image(self, img_path, light_pos, corrected_image_name, light_index):
        # Load the image
        image = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
        if image is None:
            raise ValueError(f"Cannot read image: {img_path}")

        print(f"Correcting image: {corrected_image_name} with light position: {light_pos}")
        
        # Resize to match Acq dimensions
        target_w, target_h = self.acq.image_width, self.acq.image_height
        if (image.shape[1], image.shape[0]) != (target_w, target_h):
            image = cv2.resize(image, (target_w, target_h), interpolation=cv2.INTER_AREA)

        center_distance = np.linalg.norm(np.array(light_pos))

        # Load matrices and select the current image slice
        distances_all = np.load(os.path.join(self.acq.path, "distance_matrices.npy"))
        angles_all = np.load(os.path.join(self.acq.path, "angle_matrices.npy"))

        distances = distances_all[light_index]  # shape: (height, width)
        angles = angles_all[light_index]        # shape: (height, width)

        # Calculate Kd (distance correction)
        Kd = 1 / ((distances ** 2) / (center_distance ** 2))
        old_min, old_max = np.min(Kd), np.max(Kd)
        new_min, new_max = old_min / self.DISTANCE_CORRECTION_GAIN, old_max * self.DISTANCE_CORRECTION_GAIN
        Kd = (Kd - old_min) / (old_max - old_min) * (new_max - new_min) + new_min

        # Plot Kd
        # self.plot_surface(Kd, corrected_image_name, "Kd")

        # Calculate Ka (Lambert correction)
        cos_theta = np.cos(angles)
        cos_ref = np.cos(np.arccos(-center_distance / np.mean(distances)))
        Ka = cos_theta / cos_ref
        old_min, old_max = np.min(Ka), np.max(Ka)
        new_min, new_max = old_min / self.LAMBERT_CORRECTION_GAIN, old_max * self.LAMBERT_CORRECTION_GAIN
        Ka = (Ka - old_min) / (old_max - old_min) * (new_max - new_min) + new_min

        # Plot Ka
        # self.plot_surface(Ka, corrected_image_name, "Ka")

        # Apply correction to the image
        corrected_image = image.astype(np.float32) * Kd[..., np.newaxis] * Ka[..., np.newaxis]
        corrected_image = np.clip(corrected_image, 0, 255).astype(np.uint8)

        # Save corrected image
        corrected_image_path = os.path.join(self.save_path, "results", corrected_image_name)
        cv2.imwrite(corrected_image_path, corrected_image)
        self.corrected_image_paths.append(corrected_image_path)

        # Save side-by-side comparison
        # self.plot_comparison(image, corrected_image, corrected_image_name)

    def plot_surface(self, matrix, image_name, title):
        height, width = matrix.shape
        x = np.arange(width)
        y = np.arange(height)
        X, Y = np.meshgrid(x, y)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title(title)
        ax.plot_surface(X, Y, matrix, cmap='viridis')
        eval_path = os.path.join(self.save_path, "evaluation")
        os.makedirs(eval_path, exist_ok=True)
        plot_path = os.path.join(eval_path, f"{os.path.splitext(image_name)[0]}_{title}_3D.png")
        plt.savefig(plot_path)
        plt.close()

        # 2D plot
        plt.imshow(matrix, cmap='viridis')
        plt.colorbar()
        plt.title(title)
        plot_path = os.path.join(eval_path, f"{os.path.splitext(image_name)[0]}_{title}_2D.png")
        plt.savefig(plot_path)
        plt.close()

    def plot_comparison(self, original_image, corrected_image, image_name):
        fig = plt.figure(figsize=(10, 5))
        plt.subplot(121)
        plt.imshow(cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB))
        plt.title('Original')
        plt.subplot(122)
        plt.imshow(cv2.cvtColor(corrected_image, cv2.COLOR_BGR2RGB))
        plt.title('Corrected')
        comp_path = os.path.join(self.save_path, "evaluation", f"{os.path.splitext(image_name)[0]}_comparison.png")
        plt.savefig(comp_path)
        plt.close()
