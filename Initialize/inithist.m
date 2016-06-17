function [Hist] = inithist(state, stateDeriv, Pose, Twist, Control, PropState)
    % Initialize history of the state and its derivative
    Hist.states = state;
    Hist.stateDerivs = stateDeriv;
    Hist.times = 0;

    % Initialize history of twist, pose and control structs
    Hist.poses = Pose;
    Hist.twists = Twist;
    Hist.controls = Control;
    
    Hist.propstate = PropState;

end