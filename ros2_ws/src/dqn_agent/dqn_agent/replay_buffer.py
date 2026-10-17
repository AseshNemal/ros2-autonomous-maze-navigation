"""Uniform experience replay. Bounded memory usage."""
from collections import deque
import numpy as np


class ReplayBuffer:
    def __init__(self, capacity: int = 20000, seed: int = 0):
        if capacity < 1:
            raise ValueError("capacity must be positive")
        self.buffer = deque(maxlen=capacity)
        self.rng = np.random.default_rng(seed)

    def __len__(self):
        return len(self.buffer)

    def append(self, state, action, reward, next_state, done):
        self.buffer.append((
            np.asarray(state, dtype=np.float32).copy(), int(action), float(reward),
            np.asarray(next_state, dtype=np.float32).copy(), float(bool(done))
        ))

    def sample(self, batch_size):
        if len(self.buffer) < batch_size:
            raise ValueError("Not enough experiences")
        indices = self.rng.choice(len(self.buffer), size=batch_size, replace=False)
        experiences = [self.buffer[int(i)] for i in indices]
        s, a, r, ns, d = zip(*experiences)
        return (np.stack(s), np.asarray(a, dtype=np.int64),
                np.asarray(r, dtype=np.float32), np.stack(ns),
                np.asarray(d, dtype=np.float32))
