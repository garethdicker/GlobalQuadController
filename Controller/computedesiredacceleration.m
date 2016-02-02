function [Control] = computedesiredacceleration(Control, Pose, Twist, recoveryStage)

    % Computes the desired acceleration vector. 
    global g pZ pXY dZ dXY;
 
    % TODO: why would c be set to gravity? I thought it was zero
    % TODO: sort out negatives positives
    switch recoveryStage
        case 1
            % Initialize attitude control.
            Control.acc = [0; 0; g]; %redundant
        case 2
            % Set vertical velocity gain.
            dZ = 5; % TODO: play around with this number
        case 3
            % Add vertical position and horizontal velocity control.
            pZ = 5;
            dXY = 3;
        case 4
            pZ = 5;
            dXY = 3;
            pXY = 3;
        otherwise 
            error('Invalid value for recoveryStage');
    end
    
    % Compute desired acceleration as combination of position and velocity
    % controls plus a gravity term
    
%     Control.acc = [pXY    0    0; ...
%                     0   pXY    0; ...
%                     0     0   pZ] * ([1; 1; 1] - Pose.posn) ...
    Control.acc = [dXY    0    0; ...
                    0   dXY    0; ...
                    0     0   dZ] * ([0; 0; 0] - Twist.posnDeriv) ...
                + [0; 0; g];

end