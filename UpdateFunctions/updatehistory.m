function [Hist] = updatehistory(Hist, t, state, stateDeriv)
    % Initialize history of the state and its derivative
    Hist.states = [Hist.states, state];
    Hist.stateDerivs = [Hist.stateDerivs, stateDeriv];
    Hist.t = [Hist.t, t];
end