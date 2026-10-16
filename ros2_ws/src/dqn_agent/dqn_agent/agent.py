
from pathlib import Path
import numpy as np

from .network import QNetwork
from .replay_buffer import ReplayBuffer


class DQNAgent:
    def __init__(
        self,
        state_size=15,
        action_size=5,
        gamma=0.99,
        learning_rate=0.0005,
        batch_size=32,
        warmup_steps=100,
        target_update=200,
        epsilon_start=1.0,
        epsilon_min=0.05,
        epsilon_decay_steps=20000,
        seed=42
    ):
        self.state_size = state_size
        self.action_size = action_size

        self.gamma = gamma
        self.learning_rate = learning_rate
        self.batch_size = batch_size
        self.warmup_steps = warmup_steps
        self.target_update = target_update

        self.epsilon_start = epsilon_start
        self.epsilon_min = epsilon_min
        self.epsilon_decay_steps = epsilon_decay_steps

        self.rng = np.random.default_rng(seed)

        # Network used for action selection and training
        self.online_network = QNetwork(
            input_size=state_size,
            output_size=action_size
        )

        # Separate network for calculating stable targets
        self.target_network = QNetwork(
            input_size=state_size,
            output_size=action_size
        )

        self.target_network.copy_from(
            self.online_network
        )

        self.memory = ReplayBuffer(
            capacity=20000,
            seed=seed
        )

        self.total_steps = 0
        self.training_steps = 0

    @property
    def epsilon(self):
        # Gradually reduce random exploration
        fraction = min(
            1.0,
            self.total_steps / max(1, self.epsilon_decay_steps)
        )

        return (
            self.epsilon_start
            + fraction * (self.epsilon_min - self.epsilon_start)
        )

    def choose_action(self, state, training=True):
        # Explore by selecting a random action
        if training and self.rng.random() < self.epsilon:
            return int(self.rng.integers(self.action_size))

        # Exploit by selecting the highest Q-value
        q_values = self.online_network.predict(state)

        return int(np.argmax(q_values[0]))

    def remember(
        self,
        state,
        action,
        reward,
        next_state,
        done
    ):
        self.memory.add(
            state,
            action,
            reward,
            next_state,
            done
        )

        self.total_steps += 1

    def learn(self):
        # Wait until replay memory has enough experiences
        minimum = max(
            self.batch_size,
            self.warmup_steps
        )

        if len(self.memory) < minimum:
            return None

        (
            states,
            actions,
            rewards,
            next_states,
            dones
        ) = self.memory.sample(self.batch_size)

        # Online network selects best next actions
        online_next_q = self.online_network.predict(
            next_states
        )

        best_next_actions = np.argmax(
            online_next_q,
            axis=1
        )

        # Target network evaluates selected actions
        target_next_q = self.target_network.predict(
            next_states
        )

        next_q_values = target_next_q[
            np.arange(self.batch_size),
            best_next_actions
        ]

        # Double DQN Bellman targets
        targets = (
            rewards
            + self.gamma
            * (1.0 - dones)
            * next_q_values
        )

        # Update online neural network
        loss = self.online_network.train_batch(
            states,
            actions,
            targets,
            lr=self.learning_rate
        )

        self.training_steps += 1

        # Periodically update target network
        if self.training_steps % self.target_update == 0:
            self.target_network.copy_from(
                self.online_network
            )

        return loss

    def save(self, filename):
        path = Path(filename).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)

        names = ("W1", "b1", "W2", "b2", "W3", "b3")

        weights = {
            name: getattr(self.online_network, name)
            for name in names
        }

        np.savez_compressed(
            path,
            **weights,
            state_size=self.state_size,
            action_size=self.action_size,
            total_steps=self.total_steps,
            training_steps=self.training_steps
        )

        return path

    @classmethod
    def load(cls, filename):
        with np.load(
            Path(filename).expanduser(),
            allow_pickle=False
        ) as data:
            agent = cls(
                state_size=int(data["state_size"]),
                action_size=int(data["action_size"])
            )

            names = ("W1", "b1", "W2", "b2", "W3", "b3")

            for name in names:
                stored = data[name]
                current = getattr(agent.online_network, name)

                if stored.shape != current.shape:
                    raise ValueError(
                        "Incompatible checkpoint: " + name
                    )

                current[:] = stored

            agent.target_network.copy_from(
                agent.online_network
            )

            agent.total_steps = int(data["total_steps"])
            agent.training_steps = int(data["training_steps"])

        return agent
