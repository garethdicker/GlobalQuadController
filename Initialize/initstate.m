function [state, stateDeriv] = initstate(IC)

    % Initialize full state of quadrotor.
    % Index map:
    % [1 2 3 4 5 6 7 8 9 10 11 12 13]
    % [u v w p q r x y z q0 q1 q2 q3]
    state = zeros(13,1);
    
    % Define the derivative of the state.
    stateDeriv = zeros(13,1);
    
    % Compute quaternion (q0 q1 q2 q3).
    roll = IC.attEuler(1);
    pitch = IC.attEuler(2);
    yaw = IC.attEuler(3);
    
    % Note that hover is [0 -1 0 0] with Z-axis pointing down.
    state(10:13) = angle2quat(-(roll+pi), pitch, yaw, 'xyz');
    
    % Compute rotation matrix for computing [u v w].
    R = quat2rotmat(state(10:13));
    
    % Initialize body frame velocities.
    state(1:3) = R*IC.linVel; % u v w
    % Initialize body rates.
    state(4:6) = IC.angVel;   % p q r
    % Initialize world frame position.
    state(7:9) = IC.posn;     % x y z
    
end