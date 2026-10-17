"""Targeted checks for the final-session fixes (no Gazebo needed).

1. verify_on_vm.sh slam stage: a passing map validation must not be reported as FAIL (PIPESTATUS bug).
2. set_initial_pose.py: always simulation time, and every caller in tools/ passes use_sim_time:=true.
3. tour_map_drive.py imports tf2_ros, so slam_mapping must declare tf2_ros_py.
"""
import re
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parent.parent
ROOT = PKG.parents[2]
SCRIPT = ROOT / "tools" / "verify_on_vm.sh"

# The pre-fix block, kept verbatim to prove the harness below really detects the bug.
OLD_BLOCK = '''    if [ -f "$MAPYAML" ]; then
      python3 "$WS/validate_map.py" "$MAPYAML" 2>&1 | tee -a "$OUT/v.txt" | tee -a "$LOG"
      if [ -n "${QUICK:-}" ]; then say "QUICK"; else
      [ "${PIPESTATUS[0]}" = 0 ] || fail "map validation"; fi
    fi
'''


def _validation_block():
    text = SCRIPT.read_text()
    start = text.index('    if [ -f "$MAPYAML" ]; then')
    end = text.index("\n    fi\n", start) + len("\n    fi\n")
    return text[start:end]


def _run(block, validator_rc, quick, tmp_path):
    """Run the slam-stage validation block with a stub validator; return True if fail() was called."""
    (tmp_path / "m.yaml").write_text("")
    harness = f'''set -uo pipefail
OUT={tmp_path}; TS=t; LOG={tmp_path}/log; WS={tmp_path}; MAPYAML={tmp_path}/m.yaml
say() {{ :; }}
fail() {{ echo FAIL_CALLED; }}
python3() {{ echo "RESULT: stub"; return {validator_rc}; }}
{'QUICK=1' if quick else 'unset QUICK'}
{block}
'''
    out = subprocess.run(["bash", "-c", harness], capture_output=True, text=True).stdout
    return "FAIL_CALLED" in out


def test_harness_detects_the_old_pipestatus_bug(tmp_path):
    assert _run(OLD_BLOCK, validator_rc=0, quick=False, tmp_path=tmp_path), \
        "harness should show the old block reporting FAIL for a passing validation"


def test_passing_validation_is_not_reported_as_fail(tmp_path):
    assert not _run(_validation_block(), validator_rc=0, quick=False, tmp_path=tmp_path)


def test_failing_validation_is_still_reported_as_fail(tmp_path):
    assert _run(_validation_block(), validator_rc=1, quick=False, tmp_path=tmp_path)


def test_quick_smoke_never_fails_on_validation(tmp_path):
    assert not _run(_validation_block(), validator_rc=1, quick=True, tmp_path=tmp_path)


def test_every_python_caller_of_set_initial_pose_uses_sim_time():
    callers = 0
    for sh in (ROOT / "tools").glob("*.sh"):
        text = re.sub(r"\\\n\s*", " ", sh.read_text())   # join line continuations
        for line in text.splitlines():
            if "python3" in line and "set_initial_pose.py" in line and "pkill" not in line:
                callers += 1
                assert "use_sim_time:=true" in line, (sh.name, line.strip())
    assert callers >= 2   # reset_robot.sh and verify_on_vm.sh


def test_set_initial_pose_node_uses_sim_time_by_default():
    rclpy = pytest.importorskip("rclpy")
    import importlib.util
    spec = importlib.util.spec_from_file_location("set_initial_pose_under_test", PKG / "scripts" / "set_initial_pose.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    rclpy.init()
    try:
        node = mod.InitialPose(0.25, 0.0, 0.0, 5.0)
        assert node.get_parameter("use_sim_time").value is True
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_tf2_ros_py_declared_because_tour_driver_imports_tf2_ros():
    assert "from tf2_ros import" in (PKG / "scripts" / "tour_map_drive.py").read_text()
    deps = {e.text.strip() for e in ET.parse(PKG / "package.xml").getroot().iter("exec_depend")}
    assert "tf2_ros_py" in deps
