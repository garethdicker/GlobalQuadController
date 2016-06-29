function [recoveryStage] = checkrecoverystage(Pose, Twist, recoveryStage)
    % Checks the stage of the recovery controller.
        
    attitudeStable = abs(Pose.attEuler(1)) < 0.2 && abs(Pose.attEuler(2)) < 0.2 ... % roll/pitch
                   && abs(Twist.angVel(1)) < 0.2  && abs(Twist.angVel(2)) < 0.2;    % roll/pitch rates
        
    zVelocityStable = Twist.linVel(3) < 0.3;
    
    % Not doing xy stabilization yet
%     xyVelocityStable = Twist.linVel(1) < 0.2 && Twist.linVel(2) < 0.2;
    
    if recoveryStage == 1
        if attitudeStable && zVelocityStable
            recoveryStage = 3;
        elseif attitudeStable
            recoveryStage = 2;
        end
    elseif recoveryStage == 2
        if zVelocityStable
            recoveryStage = 3;
        end
    end
end