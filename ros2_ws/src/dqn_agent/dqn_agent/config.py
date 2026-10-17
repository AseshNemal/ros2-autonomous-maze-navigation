"""Load experiment settings from YAML (config/dqn_params.yaml) with CLI overrides."""
from pathlib import Path

import yaml

DEFAULT_PATHS = [Path(__file__).resolve().parents[1] / "config" / "dqn_params.yaml"]


def load_config(path=None):
    candidates = [Path(path).expanduser()] if path else list(DEFAULT_PATHS)
    if not path:
        try:
            from ament_index_python.packages import get_package_share_directory
            candidates.append(Path(get_package_share_directory("dqn_agent")) / "config" / "dqn_params.yaml")
        except Exception:
            pass
    for c in candidates:
        if c.is_file():
            return yaml.safe_load(c.read_text())
    raise FileNotFoundError(f"DQN config not found: {candidates}")


def agent_kwargs(cfg, seed=None):
    """DQNAgent constructor arguments from the `model`/`training` sections."""
    t, m = cfg["training"], cfg["model"]
    return dict(hidden=int(m["hidden_layers"][0]), gamma=t["gamma"], learning_rate=t["learning_rate"],
                batch_size=t["batch_size"], replay_capacity=t["replay_capacity"],
                warmup=t["warmup_steps"], target_update=t["target_update_steps"],
                epsilon_start=t["epsilon_start"], epsilon_end=t["epsilon_end"],
                epsilon_decay=t["epsilon_decay_steps"], seed=seed if seed is not None else t["seed"])
