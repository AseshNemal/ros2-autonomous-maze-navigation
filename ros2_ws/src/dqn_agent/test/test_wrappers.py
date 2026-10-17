"""One-command wrappers and the committed DQN model (no ROS or Gazebo needed)."""
import hashlib
import subprocess
import sys
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parent.parent
ROOT = PKG.parents[2]
MODEL = PKG / "models" / "dqn_maze_none_60ep_0of9.npz"
MODEL_SHA256 = "d75aa1a533809e8e7075de96058ac58d6026df4afacfdb3ff8c899956b0feaf2"
sys.path.insert(0, str(PKG))


def test_committed_model_is_the_saved_60_episode_checkpoint():
    assert MODEL.is_file()
    assert hashlib.sha256(MODEL.read_bytes()).hexdigest() == MODEL_SHA256
    from dqn_agent.agent import DQNAgent
    a = DQNAgent.load(MODEL)
    assert (a.state_dim, a.action_dim) == (15, 5)
    assert (a.train_steps, a.env_steps) == (1488, 1587)
    assert a.epsilon == pytest.approx(0.4974, abs=1e-4)


def test_wrappers_parse_and_use_the_committed_model_without_training():
    for name in ("setup.sh", "run_dqn_model.sh"):
        path = ROOT / "tools" / name
        assert subprocess.run(["bash", "-n", str(path)]).returncode == 0, name
        assert path.stat().st_mode & 0o111, f"{name} is not executable"
    run = (ROOT / "tools" / "run_dqn_model.sh").read_text()
    assert "dqn_maze_none_60ep_0of9.npz" in run and "evaluate_dqn" in run
    assert "train_dqn" not in run.replace("[t]rain_dqn", "")      # evaluation only
    assert '"$HOME' not in run.replace('"$HOME/dqn_eval_results"', "")  # no hard-coded user paths


def _dummy(name):
    return subprocess.Popen(["bash", "-c", f"exec -a {name} sleep 30"])


def test_dqn_wrapper_refuses_while_nav2_is_running():
    p = _dummy("/opt/ros/jazzy/lib/nav2_dummy_for_test/node")
    try:
        import time
        time.sleep(0.3)
        r = subprocess.run(["bash", str(ROOT / "tools" / "run_dqn_model.sh")], capture_output=True, text=True, timeout=30)
        assert r.returncode == 3 and "Nav2" in r.stdout
    finally:
        p.kill()


def test_nav2_demo_refuses_while_a_dqn_run_is_active():
    p = _dummy("evaluate_dqn_dummy_for_test")
    try:
        import time
        time.sleep(0.3)
        r = subprocess.run(["bash", str(ROOT / "tools" / "demo.sh")], capture_output=True, text=True, timeout=30)
        assert r.returncode == 1 and "DQN run" in r.stdout
    finally:
        p.kill()
