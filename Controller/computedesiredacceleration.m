function [Control] = computedesiredacceleration(Control, Pose, Twist, recoveryStage)
    % Computes the desired acceleration vector. 
    global g dZ;
    
    if recoveryStage == 2 || recoveryStage == 3
        dZ = 3;
    end

    Control.acc = [0; 0; g] + [0; 0; -dZ*Twist.posnDeriv(3)]; 
    
end