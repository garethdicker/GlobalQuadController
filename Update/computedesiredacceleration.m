function [Control] = computedesiredacceleration(Control, Pose, Twist)

    % Computes the desired acceleration vector. 
    global g, pXY, pZ, dXY, dZ;
    
    
    Control.acc = pXY(Control.pose.posn(1)-
    %1) Initialize attitude control.
    
    % check if in free fall
    % need to make new variable Twist.linAccel ?
    
    % need a flag for 'entered recovery'
%     if norm(Twist.linAccel) > 8 %m/s^2
%         % initialize attitude control
%     end
    % why would c be set to gravity? I thought it was zero
    
    switch recoveryStage
        case 1
            Control.acc = [0; 0; g];
        case 2
    % GainPosition = [0; 0; 0];
    % GainVelocity = [0; 0; dZ];
    % once Twist.linVel(3) < 0.3 m/s
    
    % 3) Add vertical position control and horizontal velocity control
    % GainPosition = [0; 0; pZ];
    % GainVelocity = [dXY; dXY; dZ];
    
    % once Twist.linVel(1:2) < 0.2 m/s
   
    % 4) Add horizontal position control.
    % GainPosition = [pXY; pXY; pZ];
    % GainVelocity = [dXY; dXY; dZ];
    
 

end