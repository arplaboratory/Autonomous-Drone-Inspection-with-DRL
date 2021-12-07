import argparse
import os

import gym
import gym_ADI

from callback_buffer import CheckpointBufferCallback
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.sac import SAC
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.evaluation import evaluate_policy
from training import make_env

# Reference: https://stable-baselines3.readthedocs.io/en/master/guide/examples.html#multiprocessing-unleashing-the-power-of-vectorized-environments


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # data params
    parser.add_argument('-env_id', type=str, default='ADI-v0', choices='ADI-v0')
    parser.add_argument('-global_seed', type=int, default=1)
    parser.add_argument('-max_envs_num', type=int, default=1)
    parser.add_argument('-r_max', type=float, default=1.2)  # -1.0 means we only allow run on a sphere
    parser.add_argument('-r_min', type=float, default=0.8)
    parser.add_argument('-z_0', type=float, default=0.35)
    parser.add_argument('-max_step', type=int, default=5)
    parser.add_argument('-obs_size', type=int, default=256)

    opt = parser.parse_args()
    opt.eval = True
    opt.radius = [opt.r_min, opt.r_max]

    # Multiple env
    # env = SubprocVecEnv(
    #     [make_env(opt.env_id, i, opt.global_seed, opt.radius, opt.z_0, opt.max_step, opt.obs_size) for i in range(opt.max_envs_num)])
    # Single env
    eval_env = make_env(opt.env_id, 0, opt.global_seed, opt.radius, opt.z_0, opt.max_step, opt.obs_size, opt.eval)()

    # Check env
    check_env(eval_env)

    # model = SAC('CnnPolicy', eval_env, buffer_size=10000, verbose=1)
    model = SAC.load('./logs/final_model')

    mean_reward, std_reward = evaluate_policy(model, eval_env, n_eval_episodes=10, deterministic=True)

    print(f"mean_reward={mean_reward:.2f} +/- {std_reward}")
