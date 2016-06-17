function [recoveryStage] = checkrecoverystage(Pose, Twist)
    % Checks the stage of the recovery controller.
    
    % TODO: include acceleration check to begin
    
    attitudeStable = abs(Pose.attEuler(1)) < 0.2 && abs(Pose.attEuler(2)) < 0.2 ...
            && abs(Twist.angVel(1)) < 0.2  && abs(Twist.angVel(2)) < 0.2;
        
    zVelocityStable = Twist.linVel(3) < 0.3;
    
    xyVelocityStable = Twist.linVel(1) < 0.2 && Twist.linVel(2) < 0.2;
    
    % having stabilized height, check horizontal velocity
%     if (attitudeStable && zVelocityStable && xyVelocityStable)
%         recoveryStage = 4;
%     % having stabilized attitude, check vertical velocity
%     elseif (attitudeStable && zVelocityStable)
%         recoveryStage = 3;
    % check Euler angles and body rates
    if (attitudeStable)
        recoveryStage = 2;
    else
        recoveryStage = 1;
    end
end