function [recoveryStage] = checkrecoverystage(Pose, Twist)
    % Checks the stage of the recovery controller.

    % TODO: include acceleration check to begin
    recoveryStage = 1;

    % check Euler angles and body rates
    if (Pose.attEuler(1) < 0.35 && Pose.attEuler(2) < 0.35 ...
            && Twist.angVel(1) < 0.18  && Twist.angVel(2) < 0.18)
        recoveryStage = 2;
    end
    
    % having stabilized attitude, check vertical velocity
    if (recoveryStage == 2 && Twist.linVel(3) < 0.3)
        recoveryStage = 3;
    end
    
    % having stabilized height, check horizontal velocity
    if (recoveryStage == 3 && Twist.linVel(1) < 0.2 && Twist.linVel(2) < 0.2)
        recoveryStage = 4;
    end
    
    % recovery is achieved having entered stage 4
end