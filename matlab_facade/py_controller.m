function [phi, gripper, state_id, done] = py_controller(joints_cur, dt, reset)
%PY_CONTROLLER Persistent handle to py.sorting_controller_sim.StepController.
%   Maintains the Python FSM instance across Simulink solver steps. Pass
%   reset=1 once (e.g. on the first tick) to (re)create the controller
%   with the demo fruit queue used by the FSM slice.
%
%   Inputs
%   ------
%   joints_cur : 4x1 double, current joint angles (rad)
%   dt         : scalar double, solver step (s)
%   reset      : scalar, non-zero rebuilds the controller
%
%   Outputs
%   -------
%   phi      : 4x1 double, joint command (rad)
%   gripper  : scalar, 0=open, 1=closed
%   state_id : int32, FSM state
%   done     : int32, 1 when sorting finished

    persistent ctrl

    if isempty(ctrl) || reset ~= 0
        positions = py.list({ ...
            py.list({0.30, 0.20, 0.02}), ...
            py.list({0.25, -0.15, 0.02})});
        types = py.list({'strawberry', 'banana'});
        ctrl = py.sorting_controller_sim.StepController(positions, types);
    end

    j = py.numpy.array(joints_cur(:)');
    result = ctrl.step(j, dt);
    phi_py    = result{1};
    gripper   = double(result{2});
    state_id  = int32(result{3});
    done      = int32(result{4});
    phi = double(phi_py)';
    phi = phi(:);
end
