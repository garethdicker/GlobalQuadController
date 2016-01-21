function [Control] = computedesiredacceleration(Control, Pose, Twist, recoveryStage)

    % Computes the desired acceleration vector. 
    global g pPos dPos;
 
    % TODO: need to make new variable Twist.linAccel?
    % TODO: why would c be set to gravity? I thought it was zero
    
    switch recoveryStage
        case 1
            % Initialize attitude control.
            Control.acc = [0; 0; g];
        case 2
            % Set vertical velocity gain.
            dZ = 5; % TODO: play around with this number
            dPos(3,3) = dZ;
        case 3
            % Add vertical position control and horizontal velocity
            % control.
            pZ = 1;
            pPos(3,3) = pZ;
            dXY = 1;
            dPos(1,1) = dXY;
            dPos(2,2) = dXY;
        case 4
            % Add horizontal position control.
            pXY = 1;
            pPos(1,1) = pXY;
            pPos(2,2) = pXY;
        otherwise 
            error('Invalid value for recoveryStage');
    end
    
    % Compute desired acceleration as combination of position and velocity
    % controls plus a gravity term
    % TODO: 
    Control.acc = -pPos * (Control.pose.posn - Pose.posn) + ...
                  -dPos * (Control.twist.linVel - Twist.linVel) + ...
                  + [0; 0; g];

end