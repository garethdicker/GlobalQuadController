function [recoveryStage] = checkrecoverystage(Pose, Twist)
    % Checks the stage of the recovery controller.

    % TODO: include acceleration check to begin
    
    % check Euler angles
    if (Pose.attEuler(1) < 0.35 && Pose.attEuler(2) < 0.35 ...
            && Twist.angVel(1) < 0.18  && Twist.angVel(2) < 0.18)
        recoveryStage = 2;
    else
        recoveryStage = 1;
    end
    
%     % check vertical velocity
%     if Twist.linVel(3) < 0.5
%         recoveryStage = 3
%     end
%     
%     % check horizontal velocity
%     if Twist.linVel(1) < 0.2 && Twist.linVel(2) < 0.2
%         recoveryStage = 4
%     end
end