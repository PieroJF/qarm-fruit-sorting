"""
Fruit Sorting Controller - State Machine for autonomous and remote operation.
Works with the Quanser SDK Python API.
"""

import numpy as np
import time
from enum import Enum, auto

from qarm_kinematics import forward_kinematics, inverse_kinematics
from trajectory import cubic_trajectory

GRIP_CLOSE = 0.65
GRIP_OPEN = 0.10


class State(Enum):
    INIT = auto()
    GO_HOME = auto()
    SCAN = auto()
    SELECT_FRUIT = auto()
    APPROACH = auto()
    DESCEND = auto()
    CLOSE_GRIPPER = auto()
    ASCEND_PICK = auto()
    MOVE_TO_BASKET = auto()
    DESCEND_PLACE = auto()
    OPEN_GRIPPER = auto()
    ASCEND_PLACE = auto()
    DONE = auto()


class FruitSortingController:
    """
    Autonomous fruit sorting state machine.

    Sequences: HOME -> SCAN -> for each fruit:
        SELECT -> APPROACH -> DESCEND -> CLOSE -> ASCEND -> BASKET -> DESCEND -> OPEN -> ASCEND -> HOME
    """

    # Basket positions in QArm base frame (metres) -- ADJUST TO YOUR SETUP
    BASKETS = {
        'strawberry': np.array([0.30, -0.20, 0.05]),
        'banana':     np.array([-0.30, -0.20, 0.05]),
        'tomato':     np.array([0.00, -0.35, 0.05]),
    }

    # Key heights
    HOME_POS = np.array([0.45, 0.0, 0.49])
    SAFE_Z = 0.20
    APPROACH_Z = 0.15
    PICK_Z = 0.02
    PLACE_Z = 0.10

    # Timing
    T_TRANSIT = 2.0
    T_APPROACH = 1.0
    T_PICK = 0.8
    T_DWELL = 0.5

    def __init__(self, qarm, camera=None):
        """
        Parameters
        ----------
        qarm : QArmDriver
            Connected QArm hardware driver.
        camera : QArmCamera, optional
            Camera for fruit detection.
        """
        self.qarm = qarm
        self.camera = camera
        self.state = State.INIT
        self.fruit_queue = []
        self.current_target = None
        self.target_basket = None
        self.sorted_count = 0
        self._traj_start = 0
        self._traj_duration = 0
        self._traj_start_pos = None
        self._traj_end_pos = None

    def set_fruit_positions(self, positions, types):
        """
        Manually set detected fruit positions (bypass camera).

        Parameters
        ----------
        positions : list of np.ndarray
            List of 3D positions in base frame.
        types : list of str
            List of fruit type strings.
        """
        self.fruit_queue = [
            {'pos': np.array(p), 'type': t}
            for p, t in zip(positions, types)
        ]
        print(f"Loaded {len(self.fruit_queue)} fruits into queue")

    def run_autonomous(self, dt=0.002):
        """
        Run the full autonomous sorting loop.

        Parameters
        ----------
        dt : float
            Control loop period (seconds). Default 500Hz.
        """
        print("\n=== AUTONOMOUS FRUIT SORTING ===")
        print(f"Fruits to sort: {len(self.fruit_queue)}")
        print(f"Baskets: {list(self.BASKETS.keys())}")
        print()

        self.state = State.GO_HOME
        start_time = time.time()

        while self.state != State.DONE:
            loop_start = time.time()
            self._step(time.time())
            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        total_time = time.time() - start_time
        print(f"\n=== SORTING COMPLETE ===")
        print(f"Sorted {self.sorted_count} fruits in {total_time:.1f} seconds")

    def _step(self, t):
        """Execute one step of the state machine."""
        joints, gripper = self.qarm.read_all()
        ee_pos, _ = forward_kinematics(joints)

        if self.state == State.GO_HOME:
            self._start_move(ee_pos, self.HOME_POS, self.T_TRANSIT)
            self.state = State.INIT
            self._print_state("Moving to home")

        elif self.state == State.INIT:
            if self._move_complete(t):
                self._execute_position(self._traj_end_pos, GRIP_OPEN)
                if not self.fruit_queue:
                    self.state = State.DONE
                else:
                    self.state = State.SELECT_FRUIT

        elif self.state == State.SELECT_FRUIT:
            self.current_target = self.fruit_queue.pop(0)
            basket_name = self.current_target['type']
            self.target_basket = self.BASKETS.get(basket_name, self.BASKETS['tomato'])

            approach_pos = self.current_target['pos'].copy()
            approach_pos[2] = self.APPROACH_Z
            self._start_move(ee_pos, approach_pos, self.T_TRANSIT)
            self.state = State.APPROACH
            self._print_state(f"Approaching {self.current_target['type']}")

        elif self.state == State.APPROACH:
            if self._track_trajectory(t, gripper_val=GRIP_OPEN):
                pick_pos = self.current_target['pos'].copy()
                pick_pos[2] = self.PICK_Z
                self._start_move(ee_pos, pick_pos, self.T_PICK)
                self.state = State.DESCEND
                self._print_state("Descending to pick")

        elif self.state == State.DESCEND:
            if self._track_trajectory(t, gripper_val=GRIP_OPEN):
                self._dwell_start = time.time()
                self.state = State.CLOSE_GRIPPER
                self._print_state("Closing gripper")

        elif self.state == State.CLOSE_GRIPPER:
            self._execute_position(ee_pos, GRIP_CLOSE)
            if time.time() - self._dwell_start >= self.T_DWELL:
                ascend_pos = ee_pos.copy()
                ascend_pos[2] = self.SAFE_Z
                self._start_move(ee_pos, ascend_pos, self.T_APPROACH)
                self.state = State.ASCEND_PICK
                self._print_state("Ascending with fruit")

        elif self.state == State.ASCEND_PICK:
            if self._track_trajectory(t, gripper_val=GRIP_CLOSE):
                basket_above = self.target_basket.copy()
                basket_above[2] = self.SAFE_Z
                self._start_move(ee_pos, basket_above, self.T_TRANSIT)
                self.state = State.MOVE_TO_BASKET
                self._print_state(f"Moving to {self.current_target['type']} basket")

        elif self.state == State.MOVE_TO_BASKET:
            if self._track_trajectory(t, gripper_val=GRIP_CLOSE):
                place_pos = self.target_basket.copy()
                place_pos[2] = self.PLACE_Z
                self._start_move(ee_pos, place_pos, self.T_APPROACH)
                self.state = State.DESCEND_PLACE
                self._print_state("Descending to basket")

        elif self.state == State.DESCEND_PLACE:
            if self._track_trajectory(t, gripper_val=GRIP_CLOSE):
                self._dwell_start = time.time()
                self.state = State.OPEN_GRIPPER
                self._print_state("Opening gripper")

        elif self.state == State.OPEN_GRIPPER:
            self._execute_position(ee_pos, GRIP_OPEN)
            if time.time() - self._dwell_start >= self.T_DWELL:
                self.sorted_count += 1
                ascend_pos = ee_pos.copy()
                ascend_pos[2] = self.SAFE_Z
                self._start_move(ee_pos, ascend_pos, self.T_APPROACH)
                self.state = State.ASCEND_PLACE
                self._print_state(
                    f"Placed {self.current_target['type']} "
                    f"(#{self.sorted_count})"
                )

        elif self.state == State.ASCEND_PLACE:
            if self._track_trajectory(t, gripper_val=GRIP_OPEN):
                # Return home, then check for more fruits
                self._start_move(ee_pos, self.HOME_POS, self.T_TRANSIT)
                self.state = State.INIT
                self._print_state("Returning home")

    def _start_move(self, start, end, duration):
        """Set up a new trajectory."""
        self._traj_start = time.time()
        self._traj_duration = duration
        self._traj_start_pos = start.copy()
        self._traj_end_pos = end.copy()

    def _move_complete(self, t):
        """Check if current trajectory is complete."""
        return (time.time() - self._traj_start) >= self._traj_duration

    def _track_trajectory(self, t, gripper_val):
        """Track current trajectory and return True when done."""
        elapsed = time.time() - self._traj_start
        if elapsed >= self._traj_duration:
            self._execute_position(self._traj_end_pos, gripper_val)
            return True

        pos = cubic_trajectory(
            self._traj_start_pos, self._traj_end_pos,
            self._traj_duration, elapsed
        )
        self._execute_position(pos, gripper_val)
        return False

    def _execute_position(self, target_pos, gripper_val):
        """Compute IK and send commands to the robot."""
        phi = inverse_kinematics(target_pos, gamma=0.0)
        self.qarm.set_joints_and_gripper(phi, gripper_val)

    def _print_state(self, msg):
        """Print state transition message."""
        print(f"  [{self.state.name:20s}] {msg}")
