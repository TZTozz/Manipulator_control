%% Kinematic Simulation function
function q_new = KinematicSimulation(q, q_dot, ts, q_min, q_max)
    %
    % Inputs
    % - q current robot configuration
    % - q_dot joints velocity
    % - ts sample time
    % - q_min lower joints bound
    % - q_max upper joints bound
    %
    % Outputs
    % - q new joint configuration
    q_new = q + q_dot * ts;
    q_new = max(q_min, min(q_new, q_max));
end