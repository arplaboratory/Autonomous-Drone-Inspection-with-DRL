import gym
import gym_ADI

from stable_baselines3 import SAC
from stable_baselines3.common.evaluation import evaluate_policy

env = gym.make('ADI-v0')

model = SAC.load('final_model.zip')


