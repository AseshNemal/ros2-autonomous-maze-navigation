"""Summarize real DQN and Nav2 evaluation CSVs; never fabricate results."""
import argparse
import csv
from collections import defaultdict
from pathlib import Path


def read_rows(path):
    with Path(path).expanduser().open(newline="") as stream:
        return list(csv.DictReader(stream))


def summarize(rows, label):
    if not rows:
        raise ValueError(f"{label}: no evaluation runs")
    successes, times, distances = 0, [], []
    outcomes = defaultdict(int)
    goals = set()
    for row in rows:
        outcome = str(row.get("outcome", "")).strip().lower()
        success = outcome in ("success", "succeeded")
        successes += int(success)
        outcomes[outcome or "unknown"] += 1
        goals.add((str(row.get("goal_x")), str(row.get("goal_y"))))
        metric_valid = str(row.get("metrics_valid", "true")).strip().lower() == "true"
        distance = row.get("path_length_m") or row.get("odom_distance_m")
        if metric_valid and row.get("elapsed_sim_s") and distance:
            elapsed, length = float(row["elapsed_sim_s"]), float(distance)
            if elapsed >= 0 and length >= 0:
                times.append(elapsed)
                distances.append(length)
    return dict(method=label, runs=len(rows), successes=successes,
                success_rate=successes / len(rows), valid_metrics=len(times),
                mean_time_s=sum(times) / len(times) if times else None,
                mean_path_m=sum(distances) / len(distances) if distances else None,
                outcomes=dict(outcomes), goals=goals)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dqn", required=True)
    parser.add_argument("--nav2", required=True)
    a = parser.parse_args()
    try:
        dqn = summarize(read_rows(a.dqn), "dqn")
        nav2 = summarize(read_rows(a.nav2), "nav2")
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    for value in (dqn, nav2):
        print(f"{value['method']}: runs={value['runs']}, "
              f"success_rate={value['success_rate']:.1%}, "
              f"valid_metrics={value['valid_metrics']}, "
              f"mean_time_s={value['mean_time_s']}, "
              f"mean_path_m={value['mean_path_m']}")
        print(" outcomes:", value["outcomes"])
    if dqn["goals"] != nav2["goals"]:
        print("WARNING: Goal coordinates differ, so the comparison is not equivalent.")
    print("Verify identical maze, start poses, timeout, goal tolerances and resets. "
          "The Nav2 CSV does not encode all these settings.")


if __name__ == "__main__":
    main()
