import subprocess

import numpy as np
from PIL import Image
from gym import Env
from gym.spaces import Box


class ADIEnv(Env):
    def __init__(self, rank=0, image_size=256):
        super().__init__()
        self.image_size = image_size
        self.action_space = Box(low=-1, high=1, shape=[2], dtype=np.float32)
        self.observation_space = Box(low=0, high=255,
                                     shape=[3, self.image_size, self.image_size],
                                     dtype=np.uint8)
        self.filename = '/home/jiuhong/image.png'
        self.ros_pattern = "rosservice call /call_robot {x: {x:.1f}, y: {y:.1f}, z: {z:.1f}, yaw: {yaw:.1f}," \
                           " filename: {filename:s}, topic: '/hires/image_raw/compressed', robot: 'dragonfly12'}"
        self.max_retry_time = 10

    def step(self, action):
        obs = self.get_image_after_action(action)
        return obs

    def render(self, mode='human'):
        raise NotImplementedError

    def reset(self):
        obs = self.get_image_after_action()
        return obs

    def get_image_after_action(self, action=None):
        if action is None:
            x, y, z, yaw = 0.0, 0.0, 0.0, 0.0
        else:
            x, y, z, yaw = action
        current_retry = 0
        image = None
        while current_retry < self.max_retry_time:
            try:
                process = subprocess.Popen(
                    self.ros_pattern.format(x=x, y=y, z=z, yaw=yaw, filename=self.filename),
                    stdout=subprocess.PIPE)
                output, error = process.communicate()
                if output == 'success: True':
                    image = Image.open(self.filename)
                    break
                else:
                    raise KeyError(output)
            except KeyError:
                current_retry += 1

        if image is None:
            raise KeyError('Error: Cannot get the image after 10 retries.')
        return image
