import subprocess

import numpy as np
from PIL import Image
from gym import Env
from gym.spaces import Box
import matplotlib.pyplot as plt

class ADIEnv(Env):
    def __init__(self, rank=0, image_size=256, max_step=10):
        super().__init__()
        self.image_size = image_size
        self.rank = rank
        self.action_space = Box(low=-1, high=1, shape=[4], dtype=np.float32)
        self.observation_space = Box(low=0, high=255,
                                     shape=[3, self.image_size, self.image_size],
                                     dtype=np.uint8)
        self.filename = '/home/jiuhong/image.png'
        self.ros_pattern = "rosservice call /call_robot \"{{x: {x:.1f}, y: {y:.1f}, z: {z:.1f}, yaw: {yaw:.1f},filename: {filename:s}, topic: '/hires/image_raw/compressed', robot: 'dragonfly12'}}\""
        self.max_retry_time = 10
        self.max_step = max_step
        self.current_step = 0

    def step(self, action):
        obs = self.get_image_after_action(action)
        reward = 0
        self.current_step += 1
        if self.current_step <= self.max_step:
            done = False
        else:
            done = True
        info = None
        return obs, reward, done, info

    def render(self, mode='machine'):
        image = Image.open(self.filename)
        if mode == 'human':
            plt.imshow(np.asarray(image))
        return image

    def reset(self):
        obs = self.get_image_after_action()
        reward = 0
        self.current_step = 1
        if self.current_step <= self.max_step:
            done = False
        else:
            done = True
        info = None
        return obs, reward, done, info

    def get_image_after_action(self, action=None):
        if action is None:
            x, y, z, yaw = 0.0, 0.0, 0.0, 0.0
        else:
            x, y, z, yaw = action
        current_retry = 0
        image = None
        while current_retry < self.max_retry_time:
            try:
                process = subprocess.run(
                        self.ros_pattern.format(x=x, y=y, z=z, yaw=yaw, filename=self.filename), shell=True,
                        capture_output=True)
                if process.stdout == b'success: True\n':
                    image = Image.open(self.filename)
                    print('success')
                    break
                else:
                    raise KeyError(process.stdout)
            except KeyError:
                current_retry += 1

        if image is None:
            raise KeyError('Error: Cannot get the image after 10 retries.')
        return image
