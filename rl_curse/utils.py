import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os
import csv
import time
from stable_baselines3.common.callbacks import BaseCallback


def moving_avg(x, N=50):
    if len(x) == 0:
        return np.array([])
    cumsum = np.cumsum(np.insert(np.asarray(x, dtype=float), 0, 0))
    window = np.minimum(np.arange(1, len(x) + 1), N)
    return (cumsum[N:] - cumsum[:-N]) / window[N - 1:]


class RewardPlotCallback(BaseCallback):
    def __init__(self, ckpt_dir, plot_freq=5000, verbose=0):
        super().__init__(verbose)
        self.ckpt_dir = ckpt_dir
        self.plot_dir = os.path.join(ckpt_dir, 'reward_plots')
        os.makedirs(self.plot_dir, exist_ok=True)
        self.plot_freq = plot_freq

        self.episode_rewards = []
        self.episode_lengths = []
        self._current_ep_reward = 0.0
        self._current_ep_length = 0

        self.csv_path = os.path.join(ckpt_dir, 'training_log.csv')
        self._csv_file = None
        self._csv_writer = None
        self._start_time = time.time()
        self._episode_count = 0

    def _ensure_csv(self):
        if self._csv_writer is not None:
            return
        file_exists = os.path.exists(self.csv_path)
        self._csv_file = open(self.csv_path, 'a', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        if not file_exists:
            self._csv_writer.writerow([
                'episode', 'total_reward', 'ep_length',
                'total_timesteps', 'elapsed_sec', 'avg_reward_50',
                'actor_loss', 'critic_loss',
            ])

    def _on_step(self) -> bool:
        self._ensure_csv()

        rewards = self.locals.get('rewards', None)
        dones = self.locals.get('dones', None)

        if rewards is not None:
            self._current_ep_reward += float(rewards[0])
            self._current_ep_length += 1

        if dones is not None and dones[0]:
            self._episode_count += 1
            self.episode_rewards.append(self._current_ep_reward)
            self.episode_lengths.append(self._current_ep_length)

            avg50 = np.mean(self.episode_rewards[-50:]) if self.episode_rewards else 0.0

            actor_loss = self.model.logger.name_to_value.get('train/actor_loss', 0.0)
            critic_loss = self.model.logger.name_to_value.get('train/critic_loss', 0.0)

            self._csv_writer.writerow([
                self._episode_count,
                round(self._current_ep_reward, 3),
                self._current_ep_length,
                self.num_timesteps,
                round(time.time() - self._start_time, 1),
                round(float(avg50), 3),
                round(float(actor_loss), 6),
                round(float(critic_loss), 6),
            ])
            self._csv_file.flush()

            self._current_ep_reward = 0.0
            self._current_ep_length = 0

        if self.num_timesteps > 0 and self.num_timesteps % self.plot_freq == 0:
            self._plot_rewards()

        return True

    def _plot_rewards(self):
        if len(self.episode_rewards) < 2:
            return

        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        ax = axes[0, 0]
        ax.plot(self.episode_rewards, alpha=0.3, color='steelblue', linewidth=0.5)
        avg = moving_avg(self.episode_rewards, N=min(50, len(self.episode_rewards)))
        ax.plot(range(len(avg)), avg, color='red', linewidth=1.5, label='moving avg (50)')
        ax.set_title('Episode Reward')
        ax.set_xlabel('Episode')
        ax.set_ylabel('Reward')
        ax.legend()
        ax.grid(True, alpha=0.3)

        ax = axes[0, 1]
        ax.plot(self.episode_lengths, alpha=0.3, color='green', linewidth=0.5)
        avg_len = moving_avg(self.episode_lengths, N=min(50, len(self.episode_lengths)))
        ax.plot(range(len(avg_len)), avg_len, color='red', linewidth=1.5, label='moving avg (50)')
        ax.set_title('Episode Length')
        ax.set_xlabel('Episode')
        ax.set_ylabel('Steps')
        ax.legend()
        ax.grid(True, alpha=0.3)

        ax = axes[1, 0]
        if len(self.episode_rewards) >= 50:
            success_rate = []
            for i in range(50, len(self.episode_rewards) + 1):
                recent_rewards = self.episode_rewards[i-50:i]
                success_rate.append(np.mean([1 if r > 0 else 0 for r in recent_rewards]))
            ax.plot(range(50, len(self.episode_rewards) + 1), success_rate, color='purple')
            ax.set_title('Success Rate (50-episode window)')
            ax.set_xlabel('Episode')
            ax.set_ylabel('Success Rate')
            ax.set_ylim(0, 1)
            ax.grid(True, alpha=0.3)

        ax = axes[1, 1]
        if hasattr(self.model, 'logger') and self.model.logger is not None:
            try:
                actor_loss = self.model.logger.name_to_value.get('train/actor_loss', 0.0)
                critic_loss = self.model.logger.name_to_value.get('train/critic_loss', 0.0)
                ax.bar(['Actor Loss', 'Critic Loss'], [actor_loss, critic_loss], color=['blue', 'orange'])
                ax.set_title('Current Losses')
                ax.set_ylabel('Loss')
                ax.grid(True, alpha=0.3)
            except:
                pass

        plt.tight_layout()
        plot_path = os.path.join(self.plot_dir, f'rewards_{self.num_timesteps:08d}.png')
        plt.savefig(plot_path, dpi=150, bbox_inches='tight')
        plt.close()

        print(f"[RewardPlot] Saved plot to {plot_path}")

    def _on_training_end(self):
        if self._csv_file is not None:
            self._csv_file.close()
            self._csv_file = None


class CurriculumCallback(BaseCallback):
    def __init__(self, env, stages, switch_window=20, verbose=0):
        super().__init__(verbose)
        self.env = env
        self.stages = stages
        self.switch_window = switch_window
        self.current_stage = 0
        self.episode_lengths = []
        self.success_count = 0
        self.total_episodes = 0

    def _on_step(self) -> bool:
        dones = self.locals.get('dones', None)
        infos = self.locals.get('infos', None)

        if dones is not None and dones[0]:
            ep_len = self.locals.get('infos', [{}])[0].get('episode', {}).get('l', 0)
            self.episode_lengths.append(ep_len)
            self.total_episodes += 1

            if infos is not None and len(infos) > 0:
                info = infos[0]
                if info.get('succeeded', False):
                    self.success_count += 1

            if self.current_stage < len(self.stages) - 1:
                threshold = self.stages[self.current_stage].get('switch_threshold')
                if threshold is not None and len(self.episode_lengths) >= self.switch_window:
                    recent_avg_len = np.mean(self.episode_lengths[-self.switch_window:])
                    if recent_avg_len > threshold:
                        self.current_stage += 1
                        if hasattr(self.env, 'set_curriculum_stage'):
                            self.env.set_curriculum_stage(self.current_stage)
                        print(f"[Curriculum] Stage {self.current_stage} reached! "
                              f"Avg ep_len={recent_avg_len:.1f} > threshold={threshold}")

        return True

    def _on_training_end(self):
        if self.total_episodes > 0:
            success_rate = self.success_count / self.total_episodes
            print(f"[Curriculum] Training complete. Success rate: {success_rate:.2%} ({self.success_count}/{self.total_episodes})")