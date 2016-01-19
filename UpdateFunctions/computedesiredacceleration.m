function [Control] = computedesiredacceleration(Control, Pose, Twist)

    % Computes the desired acceleration vector. 
    global g;
    
    %1) Initialize attitude control.
    
    % check if in free fall
    % need to make new variable Twist.linAccel ?
    if Twist.linAccel < 2 %m/s^2
        % initialize attitude control
    end
    % why would c be set to gravity? I thought it was zero
    
    % once within 20 degree and 10 degrees/second
    
    % 2) Add vertical velocity control.
    % GainPosition = [0; 0; 0];
    % GainVelocity = [0; 0; dZ];
    % once Twist.linVel(3) < 0.3 m/s
    
    % 3) Add vertical position control.
    % GainPosition = [0; 0; pZ];
    % GainVelocity = [0; 0; dZ];
    
    % 4) Add horizontal velocity control. 
    % GainPosition = [0; 0; pZ];
    % GainVelocity = [dXY; dXY; dZ];
    
    % once Twist.linVel(1:2) < 0.2 m/s
   
    % 5) Add horizontal position control.
    % GainPosition = [pXY; pXY; pZ];
    % GainVelocity = [dXY; dXY; dZ];
    
 

end