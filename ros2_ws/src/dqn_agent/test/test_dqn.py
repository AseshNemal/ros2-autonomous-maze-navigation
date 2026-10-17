"""CPU-only DQN unit tests; Gazebo not required."""
import numpy as np
import pytest
from dqn_agent.network import QNetwork
from dqn_agent.replay_buffer import ReplayBuffer
from dqn_agent.agent import DQNAgent
from dqn_agent.env import ACTIONS, STATE_DIM, valid_ranges, get_observation, compute_reward
from dqn_agent.compare import summarize


def test_network_shape_and_optimizer():
    n = QNetwork(STATE_DIM, len(ACTIONS), seed=5)
    x = np.random.default_rng(9).normal(size=(32, STATE_DIM)).astype(np.float32)
    old = n.predict(x)
    loss = n.train_batch(x, np.arange(32) % len(ACTIONS), np.ones(32))
    assert np.isfinite(loss)
    assert n.adam_step == 1
    assert old.shape == (32, 5)
    assert not np.array_equal(old, n.predict(x))


def test_target_copy():
    a = QNetwork(15, 5, seed=1)
    b = QNetwork(15, 5, seed=2)
    assert not np.array_equal(a.predict(np.ones(15)), b.predict(np.ones(15)))
    b.copy_from(a)
    np.testing.assert_array_equal(a.predict(np.ones(15)), b.predict(np.ones(15)))


def test_replay_capacity_and_sampling():
    r = ReplayBuffer(capacity=3, seed=1)
    for i in range(8):
        r.append(np.full(15, i), i % 5, i, np.full(15, i + 1), i == 7)
    assert len(r) == 3
    s, a, rewards, ns, done = r.sample(2)
    assert s.shape == ns.shape == (2, 15)
    assert set(rewards) <= {5., 6., 7.}
    assert a.shape == done.shape == (2,)


def test_learning_and_checkpoint(tmp_path):
    a = DQNAgent(warmup=4, batch_size=4, target_update=2, epsilon_decay=10)
    for i in range(12):
        s = np.full(15, i / 12, dtype=np.float32)
        a.observe(s, i % 5, 0.1, s + 0.03, i % 6 == 0)
        a.learn()
    assert a.train_steps >= 2
    assert 0.05 <= a.epsilon < 1
    b = DQNAgent.load(a.save(tmp_path / "model.npz"))
    np.testing.assert_allclose(a.online.predict(np.ones(15)),
                               b.online.predict(np.ones(15)), atol=0)
    assert b.train_steps == a.train_steps


def test_observation_and_rewards():
    scan = [float("inf")] * 360
    scan[0] = 0.25
    state, distance, minimum = get_observation(scan, (0, 0), 0, (1, 0))
    assert state.shape == (STATE_DIM,)
    assert np.all(np.isfinite(state))
    assert state[-1] == pytest.approx(1.0)
    assert distance == pytest.approx(1)
    assert minimum == pytest.approx(0.25)
    assert compute_reward(1, 0.8, 1) > 0
    assert compute_reward(1, 0.8, 0.10, collision=True) == -40
    assert compute_reward(0.2, 0.1, 1, reached=True) == 80


def test_invalid_scan_and_shapes():
    assert np.allclose(valid_ranges([float("nan"), -1, float("inf")]), 3.5)
    with pytest.raises(ValueError):
        valid_ranges([])
    with pytest.raises(ValueError):
        QNetwork(15, 5).predict(np.zeros(9))


def test_comparison_rejects_missing_and_preserves_failure():
    with pytest.raises(ValueError):
        summarize([], "dqn")
    d = summarize([dict(outcome="timeout", elapsed_sim_s="4.2",
                        path_length_m="1.0", goal_x="1", goal_y="2")], "dqn")
    assert d["success_rate"] == 0
    assert d["valid_metrics"] == 1
    nav = summarize([dict(outcome="SUCCEEDED", metrics_valid="True",
                          elapsed_sim_s="3.0", odom_distance_m="0.9",
                          goal_x="1", goal_y="2")], "nav2")
    assert nav["success_rate"] == 1
    assert nav["mean_path_m"] == 0.9


def test_odom_to_world_handles_teleport_offset():
    from dqn_agent.env import odom_to_world
    import math
    # odom thinks we are at (5, 5) facing 90 deg right after a teleport to world (-1.25,-1.25) yaw 0
    ref = (5.0, 5.0, math.pi / 2)
    start = (-1.25, -1.25, 0.0)
    xy, yaw = odom_to_world((5.0, 5.0), math.pi / 2, ref, start)
    assert xy == pytest.approx((-1.25, -1.25)) and yaw == pytest.approx(0.0)
    # moving +0.5 m "forward" in odom (odom +y here) must be +x in the world
    xy, yaw = odom_to_world((5.0, 5.5), math.pi / 2, ref, start)
    assert xy == pytest.approx((-0.75, -1.25), abs=1e-9) and yaw == pytest.approx(0.0)


def test_curriculum_pairs_and_waypoints():
    from dqn_agent import maze
    layout = maze.load_layout()
    rng = maze.rng_from_seed(3)
    for _ in range(20):
        a, b = maze.sample_pair(layout, rng, 1, 3)
        assert 1 <= len(maze.cell_path(layout, a, b)) - 1 <= 3
    pairs = maze.evaluation_pairs(layout)
    assert len(pairs) == 3
    wps = maze.waypoints(layout, *pairs[0])
    assert len(wps) == len(maze.cell_path(layout, *pairs[0])) - 1 and wps[-1] == pytest.approx(maze.cell_xy(layout, pairs[0][1]))


def test_config_builds_agent():
    from dqn_agent.config import load_config, agent_kwargs
    cfg = load_config()
    assert cfg["environment"]["world"] == "custom_maze"
    a = DQNAgent(**agent_kwargs(cfg))
    assert a.gamma == cfg["training"]["gamma"]
