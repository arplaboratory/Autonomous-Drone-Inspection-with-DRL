from gym import Env
from gym.spaces import Discrete, Box
import numpy as np
import random

class ADIEnv(Env):
    def __init__(self, rank=0):
        super().__init__()
        self.action_space = None
        self.observation_space = None
        self.state = None

    def step(self, action):
        pass

    def render(self, mode='human'):
        pass

    def reset(self):
        pass
