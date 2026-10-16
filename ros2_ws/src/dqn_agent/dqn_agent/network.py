"""Small fully-connected Q-network with manual NumPy backpropagation + Adam.

CPU/ARM64 compatible; no Torch or external training service is required.
"""
import numpy as np


class QNetwork:
    KEYS = ("w1", "b1", "w2", "b2", "w3", "b3")

    def __init__(self, input_size: int, n_actions: int, hidden: int = 64, seed: int = 0):
        if input_size < 1 or n_actions < 2 or hidden < 2:
            raise ValueError("Invalid network dimensions")
        rng = np.random.default_rng(seed)
        self.input_size, self.n_actions, self.hidden = input_size, n_actions, hidden
        self.params = {
            "w1": (rng.standard_normal((input_size, hidden)) * np.sqrt(2 / input_size)).astype(np.float32),
            "b1": np.zeros(hidden, dtype=np.float32),
            "w2": (rng.standard_normal((hidden, hidden)) * np.sqrt(2 / hidden)).astype(np.float32),
            "b2": np.zeros(hidden, dtype=np.float32),
            "w3": (rng.standard_normal((hidden, n_actions)) * np.sqrt(1 / hidden)).astype(np.float32),
            "b3": np.zeros(n_actions, dtype=np.float32),
        }
        self.m = {k: np.zeros_like(v) for k, v in self.params.items()}
        self.v = {k: np.zeros_like(v) for k, v in self.params.items()}
        self.adam_step = 0

    def predict(self, x):
        x = np.asarray(x, dtype=np.float32)
        if x.ndim == 1:
            x = x[None, :]
        if x.ndim != 2 or x.shape[1] != self.input_size:
            raise ValueError("Observation shape mismatch")
        p = self.params
        h1 = np.maximum(x @ p["w1"] + p["b1"], 0)
        h2 = np.maximum(h1 @ p["w2"] + p["b2"], 0)
        return h2 @ p["w3"] + p["b3"]

    def copy_from(self, other):
        if (self.input_size, self.n_actions, self.hidden) != (other.input_size, other.n_actions, other.hidden):
            raise ValueError("Target network shape mismatch")
        for k in self.KEYS:
            self.params[k][...] = other.params[k]

    def train_batch(self, states, actions, targets, learning_rate=0.0005, clip=5.0):
        x = np.asarray(states, dtype=np.float32)
        a = np.asarray(actions, dtype=np.int64)
        y = np.asarray(targets, dtype=np.float32)
        if x.ndim != 2 or x.shape[1] != self.input_size or len(a) != len(x) or len(y) != len(x):
            raise ValueError("Invalid training batch")
        if not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):
            raise ValueError("Nonfinite training input")
        n = len(x)
        p = self.params
        z1 = x @ p["w1"] + p["b1"]
        h1 = np.maximum(z1, 0)
        z2 = h1 @ p["w2"] + p["b2"]
        h2 = np.maximum(z2, 0)
        q = h2 @ p["w3"] + p["b3"]
        diff = q[np.arange(n), a] - y
        abs_diff = np.abs(diff)
        loss = np.where(abs_diff <= 1.0, 0.5 * diff**2, abs_diff - 0.5).mean()
        gq = np.zeros_like(q)
        gq[np.arange(n), a] = np.where(abs_diff <= 1.0, diff, np.sign(diff)) / n
        grads = {}
        grads["w3"] = h2.T @ gq
        grads["b3"] = gq.sum(axis=0)
        gz2 = (gq @ p["w3"].T) * (z2 > 0)
        grads["w2"] = h1.T @ gz2
        grads["b2"] = gz2.sum(axis=0)
        gz1 = (gz2 @ p["w2"].T) * (z1 > 0)
        grads["w1"] = x.T @ gz1
        grads["b1"] = gz1.sum(axis=0)
        # Global-norm gradient clipping avoids exploding updates.
        norm = np.sqrt(sum(np.sum(g * g) for g in grads.values()))
        if norm > clip:
            grads = {k: g * (clip / (norm + 1e-8)) for k, g in grads.items()}
        self.adam_step += 1
        t = self.adam_step
        for k in self.KEYS:
            g = grads[k]
            self.m[k] = 0.9 * self.m[k] + 0.1 * g
            self.v[k] = 0.999 * self.v[k] + 0.001 * (g * g)
            mhat = self.m[k] / (1 - 0.9**t)
            vhat = self.v[k] / (1 - 0.999**t)
            p[k] -= learning_rate * mhat / (np.sqrt(vhat) + 1e-8)
        return float(loss)
