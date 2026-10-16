"""Double DQN: experience replay, target network, epsilon-greedy policy."""
from pathlib import Path
import numpy as np
from .network import QNetwork
from .replay_buffer import ReplayBuffer


class DQNAgent:
    def __init__(self, state_dim=15, action_dim=5, hidden=64,
                 gamma=0.99, learning_rate=0.0005, batch_size=32,
                 replay_capacity=20000, warmup=100, target_update=200,
                 epsilon_start=1.0, epsilon_end=0.05, epsilon_decay=20000,
                 seed=42):
        self.state_dim, self.action_dim, self.hidden = state_dim, action_dim, hidden
        self.gamma, self.learning_rate, self.batch_size = gamma, learning_rate, batch_size
        self.warmup, self.target_update = warmup, target_update
        self.epsilon_start, self.epsilon_end, self.epsilon_decay = epsilon_start, epsilon_end, epsilon_decay
        self.rng = np.random.default_rng(seed)
        self.online = QNetwork(state_dim, action_dim, hidden, seed=seed)
        self.target = QNetwork(state_dim, action_dim, hidden, seed=seed + 1)
        self.target.copy_from(self.online)
        self.replay = ReplayBuffer(replay_capacity, seed=seed)
        self.env_steps = 0
        self.train_steps = 0

    @property
    def epsilon(self):
        f = min(1.0, self.env_steps / max(1, self.epsilon_decay))
        return self.epsilon_start + f * (self.epsilon_end - self.epsilon_start)

    def act(self, obs, explore=True):
        if explore and self.rng.random() < self.epsilon:
            return int(self.rng.integers(self.action_dim))
        return int(np.argmax(self.online.predict(obs)[0]))

    def observe(self, state, action, reward, next_state, done):
        self.replay.append(state, action, reward, next_state, done)
        self.env_steps += 1

    def learn(self):
        if len(self.replay) < max(self.batch_size, self.warmup):
            return None
        s, a, r, ns, done = self.replay.sample(self.batch_size)
        # Double DQN: select actions with online net, evaluate with target net.
        next_actions = np.argmax(self.online.predict(ns), axis=1)
        next_q = self.target.predict(ns)[np.arange(len(next_actions)), next_actions]
        targets = r + self.gamma * (1 - done) * next_q
        loss = self.online.train_batch(s, a, targets, self.learning_rate)
        self.train_steps += 1
        if self.train_steps % self.target_update == 0:
            self.target.copy_from(self.online)
        return loss

    def save(self, path):
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        data = {"w_" + k: v for k, v in self.online.params.items()}
        data.update(state_dim=np.array(self.state_dim), action_dim=np.array(self.action_dim),
                    hidden=np.array(self.hidden), train_steps=np.array(self.train_steps),
                    env_steps=np.array(self.env_steps))
        np.savez_compressed(str(path), **data)
        return path

    @classmethod
    def load(cls, path):
        with np.load(Path(path).expanduser(), allow_pickle=False) as data:
            cfg = {k: int(data[k]) for k in ("state_dim", "action_dim", "hidden")}
            agent = cls(**cfg)
            for k in agent.online.KEYS:
                value = data["w_" + k]
                if value.shape != agent.online.params[k].shape:
                    raise ValueError("Checkpoint weights incompatible")
                agent.online.params[k][...] = value
            agent.target.copy_from(agent.online)
            agent.train_steps = int(data["train_steps"])
            agent.env_steps = int(data["env_steps"])
        return agent
