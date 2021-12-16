import os

from stable_baselines3.common.callbacks import CheckpointCallback


class CheckpointBufferCallback(CheckpointCallback):
    def _on_step(self) -> bool:
        if self.n_calls % self.save_freq == 0:
            path = os.path.join(self.save_path, f"{self.name_prefix}_{self.num_timesteps}_steps")
            self.model.save(path)
            final_path = os.path.join(self.save_path, f"final_model")
            buffer_path = os.path.join(self.save_path, f"buffer")
            self.model.save(final_path)
            self.model.save_replay_buffer(buffer_path)
            if self.verbose > 1:
                print(f"Saving model checkpoint to {path}")
        return True
