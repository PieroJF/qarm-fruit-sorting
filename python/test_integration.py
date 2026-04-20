"""
End-to-end integration tests against mock hardware.

Exercises the full fruit-sorting pipeline without a real QArm or camera so
regressions are caught before lab sessions. Covers:

1. Fruit detection on a synthetic BGR + depth scene.
2. Depth-to-base projection via detection_depth_mm + pixel_to_world.
3. sorting_controller.FruitSortingController FSM (blocking loop) against
   a MockQArm. Verifies full pick/place cycle, IK safety on unreachable
   fruits, gripper ramp + readback, and per-fruit Z adaptation.
4. sorting_controller_sim.StepController (Simulink-facing twin) against
   MockQArm. Verifies the stepwise API completes without exceptions for
   reachable fruits.

Run with:
    C:/Python313/python.exe python/test_integration.py
Exit code is the number of failed sections (0 = all pass).
"""

import os
import sys
import time
import traceback
import numpy as np

# Allow running from repo root or python/ dir
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import cv2
from fruit_detector import (detect_fruits, detection_depth_mm,
                             CIRCULARITY_THRESH)
from qarm_kinematics import forward_kinematics, inverse_kinematics
from sorting_controller import FruitSortingController, State, GRIP_OPEN


# ------------------------------------------------------------------------
# Mocks
# ------------------------------------------------------------------------
class MockQArm:
    """Record-only qarm: set_joints_and_gripper appends to a log, read_all
    returns the last commanded joints+grip. Tracks 'perfectly'."""

    JOINT_LIMITS = np.array([
        [-2.967, 2.967], [-1.484, 1.484],
        [-1.658, 1.309], [-2.793, 2.793],
    ])

    def __init__(self):
        self._joints = np.zeros(4, dtype=float)
        self._grip = 0.15
        self.cmd_log = []

    def read_all(self):
        return self._joints.copy(), float(self._grip)

    def set_joints_and_gripper(self, phi, g):
        phi = np.asarray(phi, dtype=float).flatten()[:4]
        self._joints = phi.copy()
        self._grip = float(g)
        self.cmd_log.append((phi.copy(), float(g)))


def synthetic_scene():
    """Return a (bgr, depth) pair with 3 red/yellow fruits in known places.

    All depths inside the plausible band [200, 700] mm so detection_depth_mm
    returns valid values. Pixels are chosen to back-project to reachable
    base-frame positions given identity T_cam_to_base.
    """
    bgr = np.zeros((720, 1280, 3), dtype=np.uint8)
    depth = np.full((720, 1280), 800, dtype=np.uint16)  # background > band

    # Strawberry (elongated red): circularity ~0.5
    cv2.ellipse(bgr, (400, 400), (40, 70), 0, 0, 360, (0, 0, 230), -1)
    cv2.ellipse(depth, (400, 400), (40, 70), 0, 0, 360, 430, -1)

    # Tomato (round red, high saturation): circularity ~0.9
    cv2.circle(bgr, (800, 400), 45, (20, 20, 240), -1)
    cv2.circle(depth, (800, 400), 45, 450, -1)

    # Banana (yellow): falls in HSV banana range
    cv2.ellipse(bgr, (640, 250), (80, 25), 0, 0, 360, (0, 230, 230), -1)
    cv2.ellipse(depth, (640, 250), (80, 25), 0, 0, 360, 400, -1)

    return bgr, depth


# ------------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------------
def test_fruit_detection():
    name = "fruit_detection"
    bgr, depth = synthetic_scene()
    dets = detect_fruits(bgr, depth)
    types = sorted(d.fruit_type for d in dets)
    assert len(dets) == 3, f"expected 3 detections, got {len(dets)}: {types}"
    assert "banana" in types, f"missing banana: {types}"
    # Red fruit classification depends on CIRCULARITY_THRESH; we don't care
    # whether the round red is strawberry or tomato, only that both red blobs
    # are detected with SOME red classification
    red_dets = [d for d in dets if d.fruit_type in ("strawberry", "tomato")]
    assert len(red_dets) == 2, f"expected 2 red blobs, got {len(red_dets)}"

    # Contour-based depth should land in plausible band
    for d in dets:
        dmm = detection_depth_mm(d, depth)
        assert dmm is not None, f"no depth for {d.fruit_type}"
        assert 350 < dmm < 500, \
            f"{d.fruit_type} depth {dmm:.0f}mm outside expected 350-500mm"
    return name, True, f"3 detections, depths within expected band"


def test_detection_depth_rejects_background():
    """If the blob is surrounded by depth values outside the band, the
    helper should return None instead of leaking a background depth."""
    name = "detection_depth_rejects_background"
    bgr = np.zeros((200, 200, 3), dtype=np.uint8)
    cv2.circle(bgr, (100, 100), 30, (0, 0, 230), -1)
    # Depth is ALL background (1200 mm) even inside the blob
    depth = np.full((200, 200), 1200, dtype=np.uint16)
    dets = detect_fruits(bgr, depth)
    assert len(dets) == 1
    dmm = detection_depth_mm(dets[0], depth)
    assert dmm is None, f"expected None, got {dmm}"
    return name, True, "correctly rejected out-of-band blob"


def test_controller_full_cycle_pick_only():
    """FruitSortingController pick_only mode: detect -> pick -> lift -> open."""
    name = "controller_pick_only"
    q = MockQArm()
    c = FruitSortingController(q, pick_only=True)
    # Compress timing so the test runs fast
    c.T_TRANSIT = 0.05
    c.T_APPROACH = 0.05
    c.T_PICK = 0.05
    c.T_DWELL = 0.02
    c.T_GRIP = 0.02
    # Two reachable fruits at distinct Z values
    positions = [np.array([0.35, 0.10, 0.13]),
                 np.array([0.30, -0.05, 0.08])]
    types = ["strawberry", "tomato"]
    c.set_fruit_positions(positions, types)
    c.state = State.GO_HOME

    t0 = time.time()
    steps = 0
    while c.state != State.DONE and steps < 10000:
        c._step(time.time())
        steps += 1
        time.sleep(0.001)
        if time.time() - t0 > 10.0:
            raise AssertionError("controller did not complete within 10s")

    assert c.sorted_count == 2, f"sorted {c.sorted_count}/2"
    assert len(q.cmd_log) > 100, f"too few commands: {len(q.cmd_log)}"
    # Gripper should sweep through intermediate values, not snap
    grips = np.array([g for _, g in q.cmd_log])
    unique = np.unique(np.round(grips, 2))
    assert len(unique) >= 5, \
        f"gripper snapped (only {len(unique)} distinct values): {unique}"
    return name, True, (f"2/2 sorted in {steps} steps, "
                        f"{len(unique)} distinct grip values")


def test_controller_skips_unreachable():
    """Unreachable fruit must be skipped via _ik_safe pre-flight, not crash."""
    name = "controller_skips_unreachable"
    q = MockQArm()
    c = FruitSortingController(q, pick_only=True)
    c.T_TRANSIT = 0.05; c.T_APPROACH = 0.05; c.T_PICK = 0.05
    c.T_DWELL = 0.02; c.T_GRIP = 0.02
    # First fruit is far outside workspace (>1 m), second is reachable
    c.set_fruit_positions(
        [np.array([2.0, 0.0, 0.5]), np.array([0.35, 0.1, 0.13])],
        ["strawberry", "tomato"])
    c.state = State.GO_HOME

    t0 = time.time()
    steps = 0
    while c.state != State.DONE and steps < 10000:
        c._step(time.time())
        steps += 1
        time.sleep(0.001)
        if time.time() - t0 > 10.0:
            raise AssertionError("controller hung on unreachable fruit")

    # Only the reachable one counts
    assert c.sorted_count == 1, f"expected 1 sort, got {c.sorted_count}"
    return name, True, "unreachable skipped, reachable sorted"


def test_controller_full_sort_mode():
    """Non-pick-only mode: full approach/pick/transit/place/release cycle."""
    name = "controller_full_sort"
    q = MockQArm()
    c = FruitSortingController(q, pick_only=False)
    c.T_TRANSIT = 0.05; c.T_APPROACH = 0.05; c.T_PICK = 0.05
    c.T_DWELL = 0.02; c.T_GRIP = 0.02
    # Override baskets to reachable positions
    c.BASKETS = {
        'strawberry': np.array([0.30, -0.15, 0.10]),
        'banana':     np.array([0.30, -0.20, 0.10]),
        'tomato':     np.array([0.25, -0.25, 0.10]),
    }
    c.set_fruit_positions(
        [np.array([0.35, 0.10, 0.13]), np.array([0.30, -0.05, 0.08])],
        ["strawberry", "tomato"])
    c.state = State.GO_HOME

    t0 = time.time()
    steps = 0
    states_seen = set()
    while c.state != State.DONE and steps < 20000:
        states_seen.add(c.state)
        c._step(time.time())
        steps += 1
        time.sleep(0.001)
        if time.time() - t0 > 15.0:
            raise AssertionError("full-sort cycle hung")

    assert c.sorted_count == 2, f"sorted {c.sorted_count}/2"
    expected_states = {State.GO_HOME, State.SELECT_FRUIT, State.APPROACH,
                       State.DESCEND, State.CLOSE_GRIPPER, State.ASCEND_PICK,
                       State.MOVE_TO_BASKET, State.DESCEND_PLACE,
                       State.OPEN_GRIPPER, State.ASCEND_PLACE}
    missing = expected_states - states_seen
    assert not missing, f"states not visited: {[s.name for s in missing]}"
    return name, True, f"2/2 sorted; all 10 transit states visited"


def test_quintic_boundary_conditions():
    """Quintic must have zero velocity AND zero acceleration at both ends
    (jerk-continuous). Numerical check via finite differences."""
    name = "quintic_boundary_conditions"
    from trajectory import quintic_trajectory
    T = 1.0
    ps = np.array([0.0, 0.0, 0.0])
    pe = np.array([1.0, -0.5, 0.3])
    h = 1e-4

    def p(t):
        return quintic_trajectory(ps, pe, T, t)

    # Velocity at endpoints (forward / backward differences)
    v0 = (p(h) - p(0)) / h
    vT = (p(T) - p(T - h)) / h
    # Acceleration at endpoints (central diff near the edge)
    a0 = (p(2 * h) - 2 * p(h) + p(0)) / (h * h)
    aT = (p(T) - 2 * p(T - h) + p(T - 2 * h)) / (h * h)

    assert np.linalg.norm(v0) < 1e-3, f"v(0) not zero: {v0}"
    assert np.linalg.norm(vT) < 1e-3, f"v(T) not zero: {vT}"
    assert np.linalg.norm(a0) < 1e-1, f"a(0) not zero: {a0}"
    assert np.linalg.norm(aT) < 1e-1, f"a(T) not zero: {aT}"

    # Endpoint reached exactly
    err = np.linalg.norm(p(T) - pe)
    assert err < 1e-9, f"p(T) != pe, err={err}"

    # Mid-point sanity (smoothstep-like should be at 0.5 * (ps+pe))
    mid = p(0.5 * T)
    expected_mid = 0.5 * (ps + pe)
    assert np.linalg.norm(mid - expected_mid) < 1e-9, \
        f"mid not halfway: got {mid}, expected {expected_mid}"
    return name, True, "v=0, a=0 at both endpoints; midpoint exact"


def test_detect_and_project_fixed_shape():
    """detect_and_project returns a fixed-size matrix even when fewer
    fruits are detected than N_MAX (required for the MATLAB/Simulink
    MATLAB Function block contract)."""
    name = "detect_and_project_matrix"
    from fruit_detector import detect_and_project, N_MAX_DEFAULT
    bgr, depth = synthetic_scene()
    intr = {'fx': 907.2, 'fy': 906.8, 'cx': 645.0, 'cy': 356.7}
    T = np.eye(4)  # identity: cam coords = base coords
    out, count = detect_and_project(bgr, depth, intr, T)
    assert out.shape == (N_MAX_DEFAULT, 5), \
        f"shape {out.shape} != ({N_MAX_DEFAULT}, 5)"
    assert 2 <= count <= 3, f"expected 2-3 detections, got {count}"
    # Each valid row has type_id in {1,2,3}
    for i in range(count):
        tid = int(out[i, 0])
        assert tid in (1, 2, 3), f"row {i} bad tid {tid}"
    # Unused rows are zeros
    for i in range(count, N_MAX_DEFAULT):
        assert np.all(out[i, :] == 0), f"row {i} not zeroed: {out[i]}"
    return name, True, f"{count} rows populated, {N_MAX_DEFAULT-count} zeroed"


def test_sim_controller():
    """StepController (Simulink-facing twin) runs without raising."""
    name = "sim_controller_step_api"
    from sorting_controller_sim import StepController, S_DONE

    positions = [np.array([0.35, 0.10, 0.13]),
                 np.array([0.30, -0.05, 0.10])]
    types = ["strawberry", "tomato"]

    sc = StepController(fruit_positions=positions, fruit_types=types)
    # Shorten timings
    sc.T_TRANSIT = 0.2
    sc.T_APPROACH = 0.1
    sc.T_PICK = 0.1
    sc.T_DWELL = 0.05

    dt = 0.01
    # Start at home joints
    joints = np.zeros(4, dtype=float)
    for _ in range(5000):
        phi, grip, state_id, done = sc.step(joints, dt)
        # Simulate perfect tracking
        joints = np.asarray(phi, dtype=float).reshape(4)
        if done:
            break
    else:
        raise AssertionError("StepController did not finish in 5000 steps")

    assert sc.state == S_DONE, f"final state {sc.state} != DONE ({S_DONE})"
    assert sc.sorted_count == 2, f"sorted {sc.sorted_count}/2"
    return name, True, f"2/2 sorted in step API"


def test_sim_controller_skips_unreachable():
    """StepController must pre-flight IK and skip unreachable fruits
    without raising out of the step() call (Simulink needs this)."""
    name = "sim_controller_skips_unreachable"
    from sorting_controller_sim import StepController, S_DONE

    # First is far outside workspace, second is reachable
    positions = [np.array([2.5, 0.0, 0.5]),
                 np.array([0.35, 0.10, 0.13])]
    types = ["strawberry", "tomato"]
    sc = StepController(fruit_positions=positions, fruit_types=types)
    sc.T_TRANSIT = 0.2; sc.T_APPROACH = 0.1; sc.T_PICK = 0.1; sc.T_DWELL = 0.05

    dt = 0.01
    joints = np.zeros(4, dtype=float)
    for _ in range(8000):
        phi, grip, state_id, done = sc.step(joints, dt)
        joints = np.asarray(phi, dtype=float).reshape(4)
        if done:
            break
    else:
        raise AssertionError("StepController hung on unreachable fruit")

    assert sc.state == S_DONE, f"final state {sc.state} != DONE"
    assert sc.sorted_count == 1, \
        f"expected 1 sort (reachable only), got {sc.sorted_count}"
    return name, True, "unreachable skipped, reachable sorted"


def test_sim_controller_uses_detected_z():
    """_compute_pick_z must use the detected fruit Z instead of PICK_Z when
    the detected value is above PICK_Z."""
    name = "sim_controller_pick_z"
    from sorting_controller_sim import StepController
    sc = StepController()
    # Floor behavior: detected 0.01 below the floor -> clamp to PICK_Z
    z_low = sc._compute_pick_z(np.array([0.3, 0.1, 0.005]))
    assert z_low == sc.PICK_Z, f"low fruit not clamped: {z_low}"
    # Above floor: use fruit_z - offset
    z_high = sc._compute_pick_z(np.array([0.3, 0.1, 0.15]))
    expected = 0.15 - sc.PICK_OFFSET
    assert abs(z_high - expected) < 1e-9, f"expected {expected}, got {z_high}"
    return name, True, "pick_z clamps floor and subtracts offset"


def test_set_gripper_ramp_extracted():
    """The shared helper ramps from `from` to `target` over `duration`
    seconds using smoothstep, and after the ramp exposes `held_grip`
    matching the settled value reported by the mocked qarm."""
    name = "set_gripper_ramp_helper"
    from sorting_controller import FruitSortingController, GRIP_OPEN, GRIP_CLOSE
    q = MockQArm()
    q.fake_grip = GRIP_OPEN  # mock tracks perfectly
    c = FruitSortingController(q)
    c.T_GRIP = 0.05  # quick
    # Precondition: helper exists on the controller class
    assert hasattr(c, "set_gripper_ramp"), "set_gripper_ramp missing"
    # Ramp from current to CLOSE
    result = c.set_gripper_ramp(GRIP_CLOSE)
    # Should have sent intermediate values, not just a single snap
    cmds = [g for _, g in q.cmd_log]
    unique = set(round(v, 2) for v in cmds)
    assert len(unique) >= 5, f"no ramp, only {unique}"
    # After settling, held_grip matches the mocked actual (which tracked)
    assert abs(c._held_grip - GRIP_CLOSE) < 0.05, \
        f"held_grip {c._held_grip} != {GRIP_CLOSE}"
    return name, True, f"ramp over {len(unique)} values, held_grip correct"


# ------------------------------------------------------------------------
# Runner
# ------------------------------------------------------------------------
TESTS = [
    test_fruit_detection,
    test_detection_depth_rejects_background,
    test_detect_and_project_fixed_shape,
    test_quintic_boundary_conditions,
    test_controller_full_cycle_pick_only,
    test_controller_skips_unreachable,
    test_controller_full_sort_mode,
    test_sim_controller,
    test_sim_controller_skips_unreachable,
    test_sim_controller_uses_detected_z,
    test_set_gripper_ramp_extracted,
]


def main():
    print(f"Running {len(TESTS)} integration tests...\n")
    results = []
    for t in TESTS:
        try:
            name, ok, detail = t()
            results.append((name, ok, detail))
            print(f"  [OK]   {name}  — {detail}")
        except Exception as ex:
            name = t.__name__.replace("test_", "")
            results.append((name, False, str(ex)))
            print(f"  [FAIL] {name}  — {ex}")
            traceback.print_exc()
    fails = sum(1 for _, ok, _ in results if not ok)
    print(f"\n{len(TESTS) - fails}/{len(TESTS)} passed.")
    return fails


if __name__ == "__main__":
    sys.exit(main())
