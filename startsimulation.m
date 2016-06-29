
%                Simulation of Quadrotor Recovery Control 
%                by Gareth Dicker and Fiona Chui
%                Last updated January 2016
%
%   Description: Simulation of quadrotor collision recovery control for
%   prediction and validation of experimental collisions of Navi 
%   quadrotor platform.

clear all;
close all;
clc;

%% Definition of Constants and Variables

% Initialize global parameters.
initparams;

% Define starting pose and twist parameters.
IC.posn     = [0; 0; 2]; % world frame position (meters) 
IC.linVel   = [0; 0; 0]; % world frame velocity (m / s)

num_iter = 20;

% Set 1: 
    % abs(p,q) < 1.5 rad/s
    % abs(r) < 0.5 rad/s
    % roll < 1.57 rad
    % abs(pitch) < 0.1 rad
    % 1.05 < yaw < 1.35
    
    % Set 2: 
    % abs(p,q) < 3 rad/s
    % abs(r) < 15 rad/s
    % abs(roll) < 1.57 rad
    % abs(pitch) < 1 rad
    % -pi < yaw < pi
    
    % Set 3: 
    % abs(p,q) < 1.5 rad/s
    % abs(r) < 1 rad/s
    % -3.14 < roll < -1.57
    % -0.05 < pitch < 0.05
    % -1.8 < yaw < -1.6
    
    %Set 4: 
    % abs(p,q) < 2 rad/s
    % abs(r) < 10 rad/s
    % -2.7 < roll < -1.7
    % -1 < pitch < 1
    % -pi < yaw < 0
    
for k = 1:num_iter

k
% Set 1
%     IC.angVel   = [3*(rand-0.5); 3*(rand-0.5); (rand-0.5)]; % body rates (radians / s)
%     IC.attEuler = [pi*rand/2; 0.2*(rand-0.5); (rand-0.5)*0.3+1.2]; % [roll; pitch; yaw] (radians)

% Set 2
%     IC.angVel   = [6*(rand-0.5); 6*(rand-0.5); 30*(rand-0.5)]; % body rates (radians / s)
%     IC.attEuler = [pi*rand/2; 2*(rand-0.5); pi*2*(rand-0.5)]; % [roll; pitch; yaw] (radians)

% Set 3
%     IC.angVel   = [3*(rand-0.5); 3*(rand-0.5); 2*(rand-0.5)]; % body rates (radians / s)
%     IC.attEuler = [rand*pi/2 - pi; (rand-0.5)/10; (rand-0.5)*0.2-1.7]; % [roll; pitch; yaw] (radians)

% Set 4
    IC.angVel   = [4*(rand-0.5); 4*(rand-0.5); 20*(rand-0.5)]; % body rates (radians / s)
    IC.attEuler = [(rand-0.5)-2.2; 2*(rand-0.5); rand*pi-pi]; % [roll; pitch; yaw] (radians)

    %%
    % Initialize state and its derivative.
    [state, stateDeriv] = initstate(IC);

    endTime = 2;  % seconds
    dt = 1 / 200; % time step (Hz)

    Control = initcontrol;
    PropState = initpropstate([-1000; 1000; -1000; 1000]);
    [Pose, Twist] = updatekinematics(state, stateDeriv);
    Hist = inithist(state, stateDeriv, Pose, Twist, Control);

    %% Simulation
    recoveryStage = 1;

    for i = 0 : dt : endTime - dt

        [recoveryStage] = checkrecoverystage(Pose, Twist, recoveryStage);

        [Control] = computedesiredacceleration(Control, Pose, Twist, recoveryStage);

        % Compute control outputs
        [Control] = controllerrecovery(dt, Pose, Twist, Control, Hist);

        % Propagate dynamics.
        options = odeset('RelTol',1e-3);
        [tODE,stateODE] = ode45(@(tODE, stateODE) ...
        dynamicsystem(tODE, stateODE, dt, Control.rpm, PropState.rpm),[i i+dt], Hist.states(:,end), options);
        [stateDeriv, PropState] = dynamicsystem(tODE(end), stateODE(end,:)', dt, Control.rpm, PropState.rpm);
        state = stateODE(end,:)';
        t = tODE(end,:)- dt;

        % Update kinematic variables from dynamics output.
        [Pose, Twist] = updatekinematics(state, stateDeriv);

        % Update history.
        Hist = updatehist(Hist, t, state, stateDeriv, Pose, Twist, Control);

    end
    %% Display Plots
    hold on;
%     plotbodyrates(Hist.times, Hist.controls, Hist.twists);

    plotangles(Hist.times, Hist.poses);

end

%% TODO: change

% plotaccelerations(Hist.times, Hist.controls, Hist.twists);
% % 
% %% 
% plotposition(Hist.times, Hist.poses);
% % 
% %% 
% % 
% % %%
% % % stateDeriv
% plotvelocity(Hist.times, Hist.twists);
% % 
% % %%
% % plotcontrolforcetorque(Hist.times, Hist.controls);
% % 
% %%
% plotcontrolrpm(Hist.times, Hist.controls);
% % 
% % %%
% % ploterrorquaternion(Hist.times, Hist.controls);
% 
%% Visualize simulation.
% simvisualization(Hist.times, Hist.states, 'na');
% 

