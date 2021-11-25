import argparse

import gym

from stable_baselines3.sac import SAC
from stable_baselines3.common.vec_env import SubprocVecEnv

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # data params
    parser.add_argument('-policy', type=str, default='cnn', choices='cnn')

    opt = parser.parse_args()

    if opt.policy == 'cnn':
        policy_name = 'CnnPolicy'

    model = SAC()

