function [Hist] = updatehist(Hist, t, state, stateDeriv, Pose, Twist, Control)
    % Initialize history of the state and its derivative
    Hist.states = [Hist.states, state];
    Hist.stateDerivs = [Hist.stateDerivs, stateDeriv];
    Hist.times = [Hist.times, t];
    
    Hist.poses = [Hist.poses; Pose];
    Hist.twists = [Hist.twists; Twist];
    Hist.controls = [Hist.controls; Control];
end