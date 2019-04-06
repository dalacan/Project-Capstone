"""
This module applies images augmentations to images to simulate different weather conditions.
"""
import cv2
import numpy as np
import random
import os

class ImageAugmentation:
    def random_brightness(self, image, min_brightness=-200, max_brightness=200):
        random_brightness = random.uniform(min_brightness, max_brightness)

        return self.apply_brightness_contrast(image, random_brightness)

    def increase_random_brightness(self, image):
        random_brightness = random.uniform(50, 200)

        return self.apply_brightness_contrast(image, random_brightness)

    def decrease_random_brightness(self, image):
        random_brightness = random.uniform(-200, -50)

        return self.apply_brightness_contrast(image, random_brightness)

    def apply_brightness_contrast(self, image, brightness=0, contrast=0):
        if brightness != 0:
            if brightness > 0:
                shadow = brightness
                highlight = 255
            else:
                shadow = 0
                highlight = 255 + brightness
            alpha_b = (highlight - shadow) / 255
            gamma_b = shadow

            buf = cv2.addWeighted(image, alpha_b, image, 0, gamma_b)
        else:
            buf = image.copy()

        if contrast != 0:
            f = 131 * (contrast + 127) / (127 * (131 - contrast))
            alpha_c = f
            gamma_c = 127 * (1 - f)

            buf = cv2.addWeighted(buf, alpha_c, buf, 0, gamma_c)

        return buf

    def add_random_shadow(self, image):
        rows, cols, _ = image.shape

        x_lo, x_hi = 0, cols

        rand_x = (rows * np.random.uniform(), cols * np.random.uniform())
        x_lo, x_hi = np.min(rand_x), np.max(rand_x)

        rand_y = (rows * np.random.uniform(), rows * np.random.uniform())
        y_lo, y_hi = np.min(rand_y), np.max(rand_y)

        shadow_mask = image.copy()
        X_msk = np.mgrid[0:rows, 0:cols][0]
        Y_msk = np.mgrid[0:rows, 0:cols][1]
        shadow_mask[((X_msk - x_lo) * (y_lo - y_hi) - (x_hi - x_lo) * (Y_msk - y_hi) >= 0), :] = 0

        shadow_weight = random.uniform(0.3, 0.7)
        return cv2.addWeighted(image, 1 - shadow_weight, shadow_mask, shadow_weight, 0)

    def add_snow(self, image):
        image_HLS = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)  ## Conversion to HLS
        image_HLS = np.array(image_HLS, dtype=np.float64)
        brightness_coefficient = 2.5
        snow_point = 140  ## increase this for more snow
        image_HLS[:, :, 1][image_HLS[:, :, 1] < snow_point] = image_HLS[:, :, 1][image_HLS[:, :,
                                                                                 1] < snow_point] * brightness_coefficient  ## scale pixel values up for channel 1(Lightness)
        image_HLS[:, :, 1][image_HLS[:, :, 1] > 255] = 255  ##Sets all values above 255 to 255
        image_HLS = np.array(image_HLS, dtype=np.uint8)
        image_RGB = cv2.cvtColor(image_HLS, cv2.COLOR_HLS2RGB)  ## Conversion to RGB
        return image_RGB

    def generate_random_droplets(self, imshape, slant, drop_length, num_droplets=1500):
        drops = []
        for i in range(num_droplets):
            if slant < 0:
                x = np.random.randint(slant, imshape[1])
            else:
                x = np.random.randint(0, imshape[1] - slant)
            y = np.random.randint(0, imshape[0] - drop_length)
            drops.append((x, y))
        return drops

    def add_rain(self, image, min_num_droplets=1000, max_num_droplets=1500):
        rain_image = image.copy()

        slant_extreme = 10
        slant = np.random.randint(-slant_extreme, slant_extreme)

        # Random droplet length
        drop_length = np.random.randint(1, 20)

        # Random droplet width
        drop_width = np.random.randint(1, 2)

        # Droplet color
        drop_color = (200, 200, 200)  ## a shade of gray

        # Number of droplets
        num_droplets = np.random.randint(min_num_droplets, max_num_droplets)

        # Generate random droplets
        rain_drops = self.generate_random_droplets(image.shape, slant, drop_length, num_droplets)

        for rain_drop in rain_drops:
            cv2.line(rain_image, (rain_drop[0], rain_drop[1]), (rain_drop[0] + slant, rain_drop[1] + drop_length),
                     drop_color, drop_width)
        rain_image = cv2.blur(rain_image, (7, 7))  ## rainy view are blurry

        brightness_coefficient = 0.7  ## rainy days are usually shady
        image_HLS = cv2.cvtColor(rain_image, cv2.COLOR_RGB2HLS)  ## Conversion to HLS
        image_HLS[:, :, 1] = image_HLS[:, :,
                             1] * brightness_coefficient  ## scale pixel values down for channel 1(Lightness)
        image_RGB = cv2.cvtColor(image_HLS, cv2.COLOR_HLS2RGB)  ## Conversion to RGB
        return image_RGB

    def add_png_overlay(self, image):
        PATH_TO_IMAGE_OVERLAYS_DIR = 'assets/overlays'
        image_overlay_list = os.listdir(PATH_TO_IMAGE_OVERLAYS_DIR)

        random_effect_idx = np.random.randint(0, len(image_overlay_list))

        image_overlay_file = PATH_TO_IMAGE_OVERLAYS_DIR + '/' + image_overlay_list[random_effect_idx]

        overlay = cv2.imread(image_overlay_file)

        height, width, _ = image.shape

        overlay = cv2.resize(overlay, (width, height), interpolation=cv2.INTER_LINEAR)

        overlay_factor = random.uniform(0.1, 0.3)

        return cv2.addWeighted(image, 1 - overlay_factor, overlay, overlay_factor, 0)

    def add_fog(self, image):
        image_shape = image.shape

        fog_mask = image.copy()

        slant_extreme = 10
        slant = np.random.randint(-slant_extreme, slant_extreme)

        # Build a randomly generate fog mask from particles
        min_num_particles = (image.shape[0] * image.shape[1]) // 2  # 25%
        max_num_particles = (image.shape[0] * image.shape[1]) // 4 * 3  # 75%
        num_particles = np.random.randint(min_num_particles, max_num_particles)

        # Generate random droplets
        particles = self.generate_random_droplets((image_shape[1], image_shape[0], image_shape[2]), slant, 1,
                                                  num_particles)

        for particle in particles:
            random_fog_color = (random.uniform(225, 250), random.uniform(225, 250), random.uniform(225, 250))
            fog_mask[particle[0], particle[1]] = random_fog_color

        fog_mask = cv2.blur(fog_mask, (20, 20))

        fog_weight = random.uniform(0.3, 0.7)
        # fog_weight = 0.7
        return cv2.addWeighted(image, 1 - fog_weight, fog_mask, fog_weight, 0)
