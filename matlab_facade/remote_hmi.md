# Remote HMI subsystem — wiring reference

Inside `FruitSorting_Hardware.slx > remote_hmi`:

## Inputs
- `sub_mode` : Manual Switch (joint vs cartesian).
- `jog_buttons` : 1x8 bus. In joint mode: [+j1, -j1, +j2, -j2, +j3, -j3, +j4, -j4]. In cartesian mode: [+x, -x, +y, -y, +z, -z, +gamma, -gamma].
- `grip_open`, `grip_close` : Push Buttons.
- `e_stop` : Dashboard Push Button (large, red, latching).
- `step_size` : Slider — 1/5/10/50 mm.

## MATLAB Function "remote_step"
One function called each solver tick (dt = 0.05 s). Pseudocode:

    function [phi_cmd, gripper_cmd, status_out] = remote_step( ...
              sub_mode, jog_buttons, grip_open, grip_close, e_stop, ...
              step_size, joints_cur)
        persistent last_joints last_gripper
        if e_stop
            phi_cmd = joints_cur; gripper_cmd = 0.15; return
        end
        if isempty(last_joints), last_joints = joints_cur; end
        if isempty(last_gripper), last_gripper = 0.15; end
        [phi_cmd, last_joints] = apply_jog( ...
            joints_cur, sub_mode, jog_buttons, step_size);
        if grip_open,  last_gripper = double(py.sorting_controller.stateflow_open_grip());  end
        if grip_close, last_gripper = double(py.sorting_controller.stateflow_close_grip()); end
        gripper_cmd = last_gripper;
        status_out = pack_status(phi_cmd, gripper_cmd, sub_mode, step_size);
    end

`apply_jog` delegates to:
- `py.remote_jog.joint_nudge(current, joint, step)` for joint sub-mode.
- `py.remote_jog.cartesian_nudge(current_xyz, axis, step)` for cartesian sub-mode (wrap + IK via `py.qarm_kinematics.inverse_kinematics`).

`pack_status` writes a JSON with mode, joints_deg, xyz_m, gripper, step, last_cmd to `logs/remote_status.json` at ~10 Hz so the OpenCV companion can poll it.

## Outputs
- `phi_cmd` (4 x 1)
- `gripper_cmd` (scalar)
- Feeds into the parent model's Multiport Switch on index `mode == 2`.
