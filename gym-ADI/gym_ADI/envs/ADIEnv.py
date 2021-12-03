import subprocess
import time

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from gym import Env
from gym.spaces import Box


class ADIEnv(Env):
    def __init__(self, rank=0, radius=None, z_0=0.33, obs_size=None, max_step=5):
        super().__init__()
        if radius is None:
            radius = [0.7, -1.0]  # r_min, r_max
        self.image_size = [480, 640]  # H, W
        if obs_size is None:
            self.obs_size = self.image_size
        elif type(obs_size) is int:
            self.obs_size = [obs_size, obs_size]
        else:
            self.obs_size = obs_size

        self.radius = radius
        self.rank = rank
        self.enable_radius_change = True if self.radius[1] != -1.0 else False  # Sphere

        if not self.enable_radius_change:
            self.action_space = Box(low=-np.pi / 2, high=np.pi / 2, shape=[2], dtype=np.float32)
        else:
            self.action_space = Box(low=-np.pi / 2, high=np.pi / 2, shape=[3], dtype=np.float32)

        self.observation_space = Box(low=0, high=255,
                                     shape=[3, self.image_size, self.image_size],
                                     dtype=np.uint8)

        self.filename = '/home/jiuhong/image.png'
        self.ros_pattern = "rosservice call /call_robot \"{{x: {x:.1f}, y: {y:.1f}, z: {z:.1f}, yaw: {yaw:.1f},filename: {filename:s}, topic: '/hires/image_raw/compressed', robot: 'dragonfly12'}}\""
        self.max_retry_time = 3
        self.max_step = max_step
        self.z_0 = z_0
        self.center_image = self.image_size[0] // 2, self.image_size[1] // 2  # Y, X
        self.current_step = 0
        self.current_polar_position = self.radius[0], 0, 0  # r, phi, theta
        self.current_score = 0

    def step(self, action):
        obs, detect = self.get_image_detect_after_action(action)

        score = self.get_score(detect)
        reward = score - self.current_score
        self.current_score = score

        self.current_step += 1
        if self.current_step < self.max_step:
            done = False
        else:
            done = True
        info = None
        return obs, reward, done, info

    def render(self, mode='machine'):
        image = Image.open(self.filename)
        image_np = np.array(image)
        if mode == 'human':
            plt.imshow(image_np)
        return image_np

    def reset(self):

        # init
        self.current_polar_position = self.radius[0], 0, 0  # r, phi, theta
        self.current_score = 0

        obs, detect = self.get_image_detect_after_action()
        score = self.get_score(detect)
        self.current_score = score
        self.current_step = 0
        return obs

    def get_image_detect_after_action(self, action=None):

        if action is not None:
            self.current_polar_position = self.get_new_position(action)  # action is delta r, delta phi and delta theta

        x, y, z, yaw = self.polar_to_cart(self.current_polar_position)

        current_retry = 0
        image = None
        detect = None  # xmin ymin xmax ymax probability(detector) xmin ymin xmax ymax(vicon)
        while image is None:
            while current_retry < self.max_retry_time:
                try:
                    process = subprocess.run(
                        self.ros_pattern.format(x=x, y=y, z=z, yaw=yaw, filename=self.filename), shell=True,
                        capture_output=True)
                    output = process.stdout.decode("utf-8").split()
                    if len(output) == 10:
                        image = Image.open(self.filename)
                        detect = output[1:]
                        break
                    else:
                        raise KeyError(process.stdout)
                except KeyError:
                    current_retry += 1
            print(f'Sleep 10s: Cannot get the image after {self.max_retry_time} retries.')
            time.sleep(10.0)

        # resize image
        image = image.resize(tuple(self.obs_size))

        return np.array(image), detect

    def polar_to_cart(self, polar_position):
        r, phi, theta = polar_position
        x = r * np.sin(theta) * np.cos(phi)
        y = r * np.sin(theta) * np.sin(phi)
        z = r * np.cos(theta) + self.z_0
        yaw = phi - np.pi
        if yaw < 0:
            yaw += 2 * np.pi
        elif yaw > 2 * np.pi:
            yaw -= 2 * np.pi

        return x, y, z, yaw

    def get_new_position(self, action):
        if not self.enable_radius_change:
            r_0, phi_0, theta_0 = self.current_polar_position
            d_phi, d_theta = action
            r_t = r_0
        else:
            r_0, phi_0, theta_0 = self.current_polar_position
            d_r, d_phi, d_theta = action
            # normalize d_r from [-pi/2, pi/2] to range
            d_r = (d_r / (np.pi / 2)) * (self.radius[1] - self.radius[0])
            r_t = r_0 + d_r

        phi_t = phi_0 + d_phi
        theta_t = theta_0 + d_theta

        # Return legal r, phi and theta
        if self.enable_radius_change:
            r_t = np.clip(r_t, self.radius[0], self.radius[1])
        if phi_t < 0:
            phi_t += 2 * np.pi
        elif phi_t > 2 * np.pi:
            phi_t -= 2 * np.pi
        theta_t = np.clip(theta_t, 0, np.pi / 2)

        return r_t, phi_t, theta_t

    def get_score(self, detect):

        xmin, ymin, xmax, ymax, prob, xmin_gt, ymin_gt, xmax_gt, ymax_gt = map(float, detect)  # 640, 480
        score = 0

        # get no gt, return 0 reward (same score)
        if xmin_gt == -1:
            return self.current_score

        # If we get a bbox, score + 1
        if prob >= 0.4:
            score += 1

        # The second part is iou of pred and gt
        pred_area = (xmax - xmin) * (ymax - ymin)
        gt_area = (xmax_gt - xmin_gt) * (ymax_gt - ymin_gt)

        left = max(xmin, xmin_gt)
        right = min(xmax, xmax_gt)
        top = max(ymin, ymin_gt)
        bottom = min(ymax, ymax_gt)

        if left < right and top < bottom:
            intersection = (right - left) * (bottom - top)
        else:
            intersection = 0
        score += intersection / (pred_area + gt_area - intersection) * 1.0

        # More score if the center of the bbox is located at the center of the image (currently use gt bbox)
        center_gt = (xmax_gt + xmin_gt) / 2, (ymax_gt + ymin_gt) / 2
        distance = np.sqrt((center_gt[0] - self.center_image[1]) ** 2 + (center_gt[1] - self.center_image[0]) ** 2)
        lower_bound = min(self.center_image) / 4
        upper_bound = min(self.center_image) / 2
        if distance <= lower_bound:
            score += 2
        else:
            score += 2 - (distance - lower_bound)/(upper_bound - lower_bound)

        return score
