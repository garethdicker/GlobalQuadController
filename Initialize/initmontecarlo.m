function [MonteCarlo] = initmontecarlo()
    % Initializes structure of Monte Carlo histories.
    IC.posn     = zeros(3,1);
    IC.linVel   = zeros(3,1);
    IC.angVel   = zeros(3,1);
    IC.attEuler = zeros(3,1); 
    [state, stateDeriv] = initstate(IC);
    Control = initcontrol;
    [Pose, Twist] = updatekinematics(state, stateDeriv);
    Hist = inithist(state, stateDeriv, Pose, Twist, Control);
    endTime = 0.5; 
    dt = 1 / 200; 
    for i = 0 : dt : endTime - dt
        Control.acc = [0; 0; 9.81];
        [recoveryStage] = checkrecoverystage(Pose, Twist);
        [Control] = controllerrecovery(dt, Pose, Twist, Control, Hist);
        options = odeset('RelTol',1e-3);
        [tODE,stateODE] = ode45(@(tODE, stateODE) ...
        dynamicsystem(tODE, stateODE, dt, Control.rpm,[0; 0; 0; 0]),[i i+dt], Hist.states(:,end), options);
        [stateDeriv, ~] = dynamicsystem(tODE(end), stateODE(end,:)', dt, Control.rpm, [0; 0; 0; 0]);
        state = stateODE(end,:)';
        t = tODE(end,:)- dt;
        [Pose, Twist] = updatekinematics(state, stateDeriv);
        Hist = updatehist(Hist, t, state, stateDeriv, Pose, Twist, Control);
    end
    MonteCarlo = Hist;
end