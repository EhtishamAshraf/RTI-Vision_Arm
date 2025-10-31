""" Homomorphic Light Correction Module
This module implements a homomorphic filter for light correction on images."""

import os
import numpy as np
import cv2
from acq_data import Acq

class HomomorphicFilter:
    def __init__(self, acq, save_path, gamma_high=1.2, gamma_low=0.8, d0=35, c=2):
        self.gamma_high = gamma_high
        self.gamma_low = gamma_low
        self.d0 = d0
        self.c = c
        self.corrected_image_paths = []

        self.save_path = save_path
        result_folder = os.path.join(self.save_path, "results")
        os.makedirs(result_folder, exist_ok=True)

        for i in range(len(acq.image_paths)):
            original_name = acq.image_names[i]
            corrected_image_name = f"lightcorrected_{original_name}"
            self.correct_image(acq.image_paths[i], corrected_image_name)

        # Save new .lp file
        self._generate_new_lp(acq)

        # New Acq instance
        self.corrected_acq = Acq(self.save_path, lp_file=self.new_lp_path)

    def correct_image(self, image_path, corrected_image_name):
        image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if image is None:
            raise FileNotFoundError(f"Could not read image at {image_path}")

        image = np.float64(image) + 1.0
        image_log = np.log1p(image)

        M, N, _ = image.shape
        y = np.arange(-M / 2, M / 2)
        x = np.arange(-N / 2, N / 2)
        X, Y = np.meshgrid(x, y)
        radius = np.sqrt(X**2 + Y**2)
        h = self.gamma_high - self.gamma_low * (1 - np.exp(-self.c * (radius / self.d0)**2))

        image_filtered = np.zeros_like(image)
        for i in range(image.shape[2]):
            fft = np.fft.fftshift(np.fft.fft2(image_log[:, :, i]))
            filtered = fft * h
            image_filtered[:, :, i] = np.real(np.fft.ifft2(np.fft.ifftshift(filtered)))

        image_exp = np.expm1(image_filtered)
        image_exp = cv2.normalize(image_exp, None, alpha=0, beta=255,
                                  norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        output_path = os.path.join(self.save_path, "results", corrected_image_name)
        cv2.imwrite(output_path, image_exp)
        self.corrected_image_paths.append(output_path)
        print(f"Saved: {output_path}")

    def _generate_new_lp(self, acq):
        original_lp_name = os.path.basename(acq.lp_file)
        self.new_lp_path = os.path.join(self.save_path, f"lightcorrected_{original_lp_name}")

        with open(acq.lp_file, 'r') as f_in, open(self.new_lp_path, 'w') as f_out:
            lines = f_in.readlines()
            f_out.write(lines[0])  # Number of lights
            for line in lines[1:]:
                img_name, *direction = line.strip().split()
                corrected_img = os.path.join("results", f"lightcorrected_{img_name}")
                f_out.write(f"{corrected_img} {' '.join(direction)}\n")
