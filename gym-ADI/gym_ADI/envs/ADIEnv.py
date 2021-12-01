import subprocess

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from gym import Env
from gym.spaces import Box


class ADIEnv(Env):
    def __init__(self, rank=0, radius=None, z_0=0.33, image_size=256, max_step=10):
        super().__init__()
        if radius is None:
            radius = [0.7, -1.0]  # r_min, r_max
        self.radius = radius
        self.image_size = image_size
        self.rank = rank
        self.enable_radius_change = True if self.radius[1] != -1.0 else False  # Sphere

        if not self.enable_radius_change:
            self.action_space = Box(low=0, high=np.pi / 2, shape=[2], dtype=np.float32)
        else:
            self.action_space = Box(low=0, high=np.pi / 2, shape=[3], dtype=np.float32)

        self.observation_space = Box(low=0, high=255,
                                     shape=[3, self.image_size, self.image_size],
                                     dtype=np.uint8)

        self.filename = '/home/jiuhong/image.png'
        self.ros_pattern = "rosservice call /call_robot \"{{x: {x:.1f}, y: {y:.1f}, z: {z:.1f}, yaw: {yaw:.1f},filename: {filename:s}, topic: '/hires/image_raw/compressed', robot: 'dragonfly12'}}\""
        self.max_retry_time = 10
        self.max_step = max_step
        self.z_0 = z_0
        self.current_step = 0
        self.current_polar_position = self.radius[0], 0, 0  # r, phi, theta

        self.last_score = 0

    def step(self, action):
        obs = self.get_image_after_action(action)

        score = self.get_score(obs)
        reward = score - self.last_score
        self.last_score = score

        self.current_step += 1
        if self.current_step <= self.max_step:
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
        obs = self.get_image_after_action()

        score = self.get_score(obs)
        reward = score - self.last_score
        self.last_score = score

        self.current_step = 1
        if self.current_step <= self.max_step:
            done = False
        else:
            done = True
        info = None
        return obs, reward, done, info

    def get_image_after_action(self, action=None):

        if action is not None:
            self.current_polar_position = self.get_new_position(action)  # action is delta r, delta phi and delta theta

        x, y, z, yaw = self.polar_to_cart(self.current_polar_position)

        current_retry = 0
        image = None
        while current_retry < self.max_retry_time:
            try:
                process = subprocess.run(
                    self.ros_pattern.format(x=x, y=y, z=z, yaw=yaw, filename=self.filename), shell=True,
                    capture_output=True)
                if process.stdout == b'success: True\n':
                    image = Image.open(self.filename)
                    break
                else:
                    raise KeyError(process.stdout)
            except KeyError:
                current_retry += 1

        if image is None:
            raise KeyError('Error: Cannot get the image after 10 retries.')
        return np.array(image)

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
            # normalize d_r from [0, pi/2] to radius
            d_r = (d_r / (np.pi / 2)) * (self.radius[1] - self.radius[0]) + self.radius[0]
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

    def get_score(self, obs):
        return 0
