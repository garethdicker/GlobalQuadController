function [recoveryStage] = checkrecoverystage(Pose, Twist, recoveryStage)
    % Checks the stage of the recovery controller.
    
    % TODO: include acceleration check to begin
    
    attitudeStable = Pose.attEuler(1) < 0.35 && Pose.attEuler(2) < 0.35 ...
            && Twist.angVel(1) < 0.18  && Twist.angVel(2) < 0.18;
        
    zVelocityStable = Twist.linVel(3) < 0.2;
    
    xyVelocityStable = Twist.linVel(1) < 0.2 && Twist.linVel(2) < 0.2;
    
    switch recoveryStage
        case 1
            if attitudeStable
                recoveryStage = 2;
            end
        case 2
            if zVelocityStable
               recoveryStage = 3;
            end
        case 3
            if xyVelocityStable
                recoveryStage = 4;
            end
        case 4
            return
        otherwise 
            error('Invalid recovery stage!');
    end
end