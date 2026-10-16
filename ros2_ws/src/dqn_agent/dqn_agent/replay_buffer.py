
from collections import deque
import numpy as np


class ReplayBuffer:
    def __init__(self, capacity=20000, seed=42):
        if capacity <= 0:
            raise ValueError("Capacity must be positive")

        self.memory = deque(maxlen=capacity)
        self.rng = np.random.default_rng(seed)

    def add(self, state, action, reward, next_state, done):
        experience = (
            np.asarray(state, dtype=np.float32).copy(),
            int(action),
            float(reward),
            np.asarray(next_state, dtype=np.float32).copy(),
            bool(done)
        )

        self.memory.append(experience)

    def sample(self, batch_size=32):
        if batch_size <= 0:
            raise ValueError("Batch size must be positive")

        if len(self.memory) < batch_size:
            raise ValueError("Not enough experiences in memory")

        indices = self.rng.choice(
            len(self.memory),
            size=batch_size,
            replace=False
        )

        batch = [self.memory[int(i)] for i in indices]

        states, actions, rewards, next_states, dones = zip(*batch)

        return (
            np.stack(states),
            np.asarray(actions, dtype=np.int64),
            np.asarray(rewards, dtype=np.float32),
            np.stack(next_states),
            np.asarray(dones, dtype=np.float32)
        )

    def __len__(self):
        return len(self.memory)
