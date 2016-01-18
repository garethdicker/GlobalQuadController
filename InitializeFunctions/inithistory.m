function [Hist] = inithistory(state, stateDeriv, Twist, Pose, Control)
    % Initialize history of the state and its derivative
    Hist.states = state;
    Hist.stateDerivs = stateDeriv;
    Hist.t = 0;

    % Initialize history of twist, pose and control structs
    Hist.poses = Pose;
    Hist.twists = Twist;
    Hist.controls = Control;

end