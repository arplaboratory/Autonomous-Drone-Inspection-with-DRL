import argparse
import os

import gym
import gym_ADI

from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.sac import SAC
from stable_baselines3.common.env_checker import check_env

# Reference: https://stable-baselines3.readthedocs.io/en/master/guide/examples.html#multiprocessing-unleashing-the-power-of-vectorized-environments

def make_env(env_id, rank, seed, radius, z_0, max_step, obs_size):
    """
    Utility function for multiprocessed env.
    :param z_0: z_0
    :param radius: [r_min, r_max]
    :param env_id: (str) the environment ID
    :param seed: (int) the inital seed for RNG
    :param rank: (int) index of the subprocess
    """

    def _init():
        env = gym.make(env_id, seed=seed, rank=rank, radius=radius, z_0=z_0, max_step=max_step, obs_size=obs_size)
        env.seed(seed + rank)
        return env

    set_random_seed(seed)
    return _init


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # data params
    parser.add_argument('-env_id', type=str, default='ADI-v0', choices='ADI-v0')
    parser.add_argument('-policy', type=str, default='cnn', choices='cnn')
    parser.add_argument('-global_seed', type=int, default=1)
    parser.add_argument('-max_envs_num', type=int, default=1)
    parser.add_argument('-batch_size', type=int, default=64)
    parser.add_argument('-r_max', type=float, default=1.2)  # -1.0 means we only allow run on a sphere
    parser.add_argument('-r_min', type=float, default=0.8)
    parser.add_argument('-z_0', type=float, default=0.35)
    parser.add_argument('-max_step', type=int, default=5)
    parser.add_argument('-obs_size', type=int, default=256)
    parser.add_argument('-buffer_size', type=int, default=100000)
    parser.add_argument('-total_timesteps', type=int, default=25000)

    opt = parser.parse_args()
    opt.radius = [opt.r_min, opt.r_max]

    # Multiple env
    # env = SubprocVecEnv(
    #     [make_env(opt.env_id, i, opt.global_seed, opt.radius, opt.z_0, opt.max_step, opt.obs_size) for i in range(opt.max_envs_num)])
    # Single env
    env = make_env(opt.env_id, 0, opt.global_seed, opt.radius, opt.z_0, opt.max_step, opt.obs_size)()

    if opt.policy == 'cnn':
        policy_name = 'CnnPolicy'
    else:
        raise KeyError
        
    # Check env
    check_env(env)

    eval_callback = EvalCallback(env, best_model_save_path='./logs/',
                                 log_path='./logs/', eval_freq=1000,
                                 deterministic=True, render=False)

    model = SAC(policy_name, env, verbose=1, buffer_size=opt.buffer_size, batch_size=opt.batch_size,
                tensorboard_log="./tb/")

    if os.path.isfile('./logs/final_model.pth'):
        model.load('./logs/final_model.pth')
    if os.path.isfile('./buffer.pth'):
        model.load_replay_buffer('./buffer.pth')

    try:
        model.learn(total_timesteps=opt.total_timesteps, callback=eval_callback)
    finally:
        # Save Replay Buffer
        if model.num_timesteps > 1000:
            model.save('./logs/final_model.pth')
            model.save_replay_buffer('./buffer.pth')
