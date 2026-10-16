
import numpy as np


class QNetwork:
    def __init__(self, input_size=15, output_size=5, hidden=64):
        self.input_size = input_size
        self.output_size = output_size

        rng = np.random.default_rng(42)

        # Initialize network weights and biases
        self.W1 = (
            rng.standard_normal((input_size, hidden))
            * np.sqrt(2 / input_size)
        ).astype(np.float32)
        self.b1 = np.zeros(hidden, dtype=np.float32)

        self.W2 = (
            rng.standard_normal((hidden, hidden))
            * np.sqrt(2 / hidden)
        ).astype(np.float32)
        self.b2 = np.zeros(hidden, dtype=np.float32)

        self.W3 = (
            rng.standard_normal((hidden, output_size))
            * np.sqrt(1 / hidden)
        ).astype(np.float32)
        self.b3 = np.zeros(output_size, dtype=np.float32)

    def forward(self, state):
        x = np.asarray(state, dtype=np.float32)

        if x.ndim == 1:
            x = x.reshape(1, -1)

        if x.ndim != 2 or x.shape[1] != self.input_size:
            raise ValueError("Invalid state dimensions")

        # Hidden layer 1: ReLU activation
        self.x = x
        self.z1 = x @ self.W1 + self.b1
        self.a1 = np.maximum(0, self.z1)

        # Hidden layer 2: ReLU activation
        self.z2 = self.a1 @ self.W2 + self.b2
        self.a2 = np.maximum(0, self.z2)

        # Output: Q-values for all actions
        q_values = self.a2 @ self.W3 + self.b3

        return q_values

    def predict(self, state):
        return self.forward(state)

    def train_batch(self, states, actions, targets, lr=0.0005):
        # Forward propagation
        q_values = self.forward(states)

        actions = np.asarray(actions, dtype=np.int64)
        targets = np.asarray(targets, dtype=np.float32)
        batch_size = len(actions)

        if (
            batch_size == 0
            or q_values.shape[0] != batch_size
            or targets.shape != (batch_size,)
            or np.any(actions < 0)
            or np.any(actions >= self.output_size)
        ):
            raise ValueError("Invalid training batch")

        selected_q = q_values[
            np.arange(batch_size), actions
        ]

        error = selected_q - targets

        # Huber loss
        abs_error = np.abs(error)
        loss = np.mean(
            np.where(
                abs_error <= 1,
                0.5 * error**2,
                abs_error - 0.5
            )
        )

        # Gradient of Huber loss
        grad_q = np.zeros_like(q_values)
        grad_q[np.arange(batch_size), actions] = (
            np.where(abs_error <= 1, error, np.sign(error))
            / batch_size
        )

        # Backpropagation: output layer
        dW3 = self.a2.T @ grad_q
        db3 = np.sum(grad_q, axis=0)

        # Backpropagation: hidden layer 2
        dz2 = (grad_q @ self.W3.T) * (self.z2 > 0)
        dW2 = self.a1.T @ dz2
        db2 = np.sum(dz2, axis=0)

        # Backpropagation: hidden layer 1
        dz1 = (dz2 @ self.W2.T) * (self.z1 > 0)
        dW1 = self.x.T @ dz1
        db1 = np.sum(dz1, axis=0)

        # Gradient clipping
        gradients = [dW1, db1, dW2, db2, dW3, db3]
        norm = np.sqrt(sum(np.sum(g**2) for g in gradients))
        scale = min(1.0, 5.0 / (norm + 1e-8))

        # Gradient descent weight updates
        self.W1 -= lr * scale * dW1
        self.b1 -= lr * scale * db1

        self.W2 -= lr * scale * dW2
        self.b2 -= lr * scale * db2

        self.W3 -= lr * scale * dW3
        self.b3 -= lr * scale * db3

        return float(loss)

    def copy_from(self, other):
        # Copy weights into a target network
        for name in ("W1", "b1", "W2", "b2", "W3", "b3"):
            getattr(self, name)[:] = getattr(other, name)

