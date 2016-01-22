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
    
    endTime = 0.5;  % seconds
    dt = 1 / 200; % time step (Hz)

    % Simulation
    for i = 0 : dt : endTime - dt
        
        % Set control input for recovery controller.
        % TODO: make world frame not body
        Control.acc = [0; 0; 9.81];

    %     [recoveryStage] = checkrecoverystage(Pose, Twist);

    %     [Control] = computedesiredacceleration(Control, Pose, Twist, recoveryStage);

        % Compute control outputs
        [Control] = controllerrecovery(dt, Pose, Twist, Control, Hist);

        % Propagate dynamics.
        options = odeset('RelTol',1e-3);
        [tODE,stateODE] = ode45(@(tODE, stateODE) ...
        dynamicsystem(tODE, stateODE, Control),[i i+dt], Hist.states(:,end), options);
        [stateDeriv] = dynamicsystem(tODE(end), stateODE(end,:)', Control);
        state = stateODE(end,:)';
        t = tODE(end,:)- dt;

        % Update kinematic variables from dynamics output.
        [Pose, Twist] = updatekinematics(state, stateDeriv);

        % Update history.
        Hist = updatehist(Hist, t, state, stateDeriv, Pose, Twist, Control);

    end
    
    MonteCarlo = Hist;

end