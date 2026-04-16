"""
Trajectory generation for the QArm.
Cubic splines with zero-velocity boundary conditions.
"""

import numpy as np


def cubic_trajectory(p_start, p_end, T, t):
    """
    Evaluate cubic spline trajectory at time t.
    Zero velocity at start and end.

    Parameters
    ----------
    p_start : np.ndarray  - Start position (3,)
    p_end   : np.ndarray  - End position (3,)
    T       : float       - Segment duration (seconds)
    t       : float       - Current time [0, T]

    Returns
    -------
    pos : np.ndarray - Interpolated position (3,)
    """
    t = np.clip(t, 0, T)
    a0 = p_start
    a2 = 3 * (p_end - p_start) / T**2
    a3 = -2 * (p_end - p_start) / T**3
    return a0 + a2 * t**2 + a3 * t**3


def multi_segment_trajectory(waypoints, durations, t):
    """
    Navigate through multiple waypoints using cubic splines.

    Parameters
    ----------
    waypoints  : np.ndarray - (N, 3) array of waypoints
    durations  : list/array - (N-1,) segment durations
    t          : float      - Current time

    Returns
    -------
    pos         : np.ndarray - (3,) current position
    segment_idx : int        - Which segment we're in
    """
    total_time = sum(durations)
    t = np.clip(t, 0, total_time)

    cumulative = 0.0
    for i, dur in enumerate(durations):
        if t <= cumulative + dur:
            seg_time = t - cumulative
            return cubic_trajectory(waypoints[i], waypoints[i+1], dur, seg_time), i
        cumulative += dur

    return waypoints[-1].copy(), len(durations) - 1


def smooth_move(start_joints, end_joints, duration, dt=0.002):
    """
    Generate a smooth joint-space trajectory.

    Parameters
    ----------
    start_joints : np.ndarray - (4,) start joint angles
    end_joints   : np.ndarray - (4,) end joint angles
    duration     : float      - Duration in seconds
    dt           : float      - Time step

    Yields
    ------
    np.ndarray - (4,) interpolated joint angles at each time step
    """
    steps = int(duration / dt)
    for i in range(steps + 1):
        t = i * dt
        s = 3 * (t/duration)**2 - 2 * (t/duration)**3  # cubic ease
        yield start_joints + s * (end_joints - start_joints)
