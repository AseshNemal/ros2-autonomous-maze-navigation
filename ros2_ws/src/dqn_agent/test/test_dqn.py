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


def test_waypoint_tolerance_is_less_than_half_a_cell_and_matches_config():
    """A tolerance of half a cell (0.25 m) switches the target on cell ENTRY and cuts corners."""
    import inspect
    from dqn_agent.config import load_config
    from dqn_agent.env import GazeboDQNEnv
    cfg = load_config()["environment"]
    default = inspect.signature(GazeboDQNEnv.__init__).parameters["waypoint_tolerance"].default
    assert cfg["waypoint_tolerance_m"] == default
    assert cfg["waypoint_tolerance_m"] < 0.25
    assert cfg["waypoint_tolerance_m"] >= 0.07   # still reachable with odometry/step quantisation (0.04 m/step)


def test_reward_not_penalised_at_corridor_centre():
    """Centred in a 0.5 m cell the side walls are ~0.245 m away: driving well must pay > 0."""
    from dqn_agent.env import NEAR_OBSTACLE_M
    assert NEAR_OBSTACLE_M < 0.245
    assert compute_reward(1.0, 0.96, 0.245) == pytest.approx(12 * 0.04 - 0.05)
    assert compute_reward(1.0, 1.0, NEAR_OBSTACLE_M - 0.01) < compute_reward(1.0, 1.0, 0.245)


def test_lidar_neg_inf_is_close_not_free():
    v = valid_ranges([float("-inf"), float("inf"), 1.0])
    assert v[0] == pytest.approx(0.12) and v[1] == pytest.approx(3.5) and v[2] == pytest.approx(1.0)


def test_bellman_target_uses_done_mask_and_target_net():
    a = DQNAgent(warmup=1, batch_size=2, gamma=0.5, target_update=10**6, seed=3)
    s = np.zeros(15, dtype=np.float32)
    ns = np.ones(15, dtype=np.float32)
    a.target.params["b3"] += np.array([0, 0, 10.0, 0, 0], dtype=a.target.params["b3"].dtype)  # target net differs
    captured = {}
    orig = a.online.train_batch

    def spy(states, actions, targets, lr):
        captured["t"] = targets.copy()
        return orig(states, actions, targets, lr)
    a.online.train_batch = spy
    a.observe(s, 0, 1.0, ns, True)    # terminal: target is the reward only
    a.observe(s, 1, 1.0, ns, False)   # non-terminal: reward + gamma * target-net Q(ns, argmax online)
    a.rng = np.random.default_rng(0)
    a.replay.sample = lambda n: tuple(np.asarray(x) for x in (
        np.stack([s, s]), [0, 1], [1.0, 1.0], np.stack([ns, ns]), [1.0, 0.0]))
    a.learn()
    best = int(np.argmax(a.online.predict(ns)[0]))
    expected_next = float(a.target.predict(ns)[0][best])
    assert captured["t"][0] == pytest.approx(1.0)
    assert captured["t"][1] == pytest.approx(1.0 + 0.5 * expected_next)
    assert a.train_steps == 1   # an optimizer update really happened


def test_checkpoint_restores_hyperparameters_and_epsilon(tmp_path):
    a = DQNAgent(epsilon_decay=3000, gamma=0.9, batch_size=16, warmup=7, target_update=33, seed=1)
    a.env_steps = 1500
    path = a.save(tmp_path / "ckpt")          # no suffix: save and load must agree on the file
    assert path.suffix == ".npz" and path.is_file()
    b = DQNAgent.load(tmp_path / "ckpt")
    assert (b.epsilon_decay, b.gamma, b.batch_size, b.warmup, b.target_update) == (3000, 0.9, 16, 7, 33)
    assert b.epsilon == pytest.approx(a.epsilon)


def test_default_training_pair_is_the_shortest_goal():
    from dqn_agent import maze
    from dqn_agent.train import arguments, pick_pair
    layout = maze.load_layout()
    a, b = pick_pair(layout, arguments([]), maze.rng_from_seed(0), 1)
    lengths = [len(maze.cell_path(layout, *p)) for p in maze.evaluation_pairs(layout)]
    assert len(maze.cell_path(layout, a, b)) == min(lengths)


def test_time_limit_truncation_is_not_stored_as_terminal():
    import inspect
    from dqn_agent import train
    src = inspect.getsource(train.run)
    assert "agent.observe(state, action, reward, next_state, terminated)" in src
