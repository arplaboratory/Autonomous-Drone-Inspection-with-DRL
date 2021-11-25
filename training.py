import argparse

import gym

from stable_baselines3.sac import SAC
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.common.vec_env import SubprocVecEnv


# Reference: https://stable-baselines3.readthedocs.io/en/master/guide/examples.html#multiprocessing-unleashing-the-power-of-vectorized-environments

def make_env(env_id, rank, seed=0):
    """
    Utility function for multiprocessed env.

    :param env_id: (str) the environment ID
    :param num_env: (int) the number of environments you wish to have in subprocesses
    :param seed: (int) the inital seed for RNG
    :param rank: (int) index of the subprocess
    """

    def _init():
        env = gym.make(env_id, rank=rank)
        env.seed(seed + rank)
        return env

    set_random_seed(seed)
    return _init


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # data params
    parser.add_argument('-env_id', type=str, default='ADI-v0', choice='ADI-v0')
    parser.add_argument('-policy', type=str, default='cnn', choices='cnn')
    parser.add_argument('-max_envs_num', type=int, default=1)

    opt = parser.parse_args()

    env = SubprocVecEnv([make_env(opt.env_id, i) for i in range(opt.max_envs_num)])

    if opt.policy == 'cnn':
        policy_name = 'CnnPolicy'
    else:
        raise KeyError

    model = SAC(policy_name, env, verbose=1)
    model.learn(total_timesteps=25000)
