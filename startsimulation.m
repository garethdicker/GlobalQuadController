
%                Simulation of Quadrotor Recovery Control 
%                by Gareth Dicker and Fiona Chui
%                Last updated January 2016
%
%   Description: Simulation of quadrotor collision recovery control for
%   prediction and validation of experimental collisions of the Spiri 
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
IC.angVel   = [30; 30; 4]; % body rates (radians / s)
IC.attEuler = [pi; 0; 0]; % [roll; pitch; yaw] (radians)

%%
% Initialize state and its derivative.
[state, stateDeriv] = initstate(IC);

endTime = 1;  % seconds
dt = 1 / 200; % time step (Hz)

% Define control types using pose and twist struct types.
Control = initcontrol;

% Initialize propeller state
PropState = initpropstate([-1000; 1000; -1000; 1000]);

% Update pose and twist structs from state and its derivative.
[Pose, Twist] = updatekinematics(state, stateDeriv);

% Initialize history for plotting and visualizing simulation.
Hist = inithist(state, stateDeriv, Pose, Twist, Control);

%% Simulation
for i = 0 : dt : endTime - dt
    
    % Set control input for recovery controller.
    % TODO: make world frame not body
    Control.acc = [0; 0; 9.81];
    
    [recoveryStage] = checkrecoverystage(Pose, Twist);
    
%     [Control] = computedesiredacceleration(Control, Pose, Twist, recoveryStage);
    
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
% plotbodyrates(Hist.times, Hist.controls, Hist.twists);

%% TODO: change
% plotaccelerations(Hist.times, Hist.controls, Hist.stateDerivs(1:3,:), Hist.states(10:13,:));
% 
% %% 
% plotposition(Hist.times, Hist.poses);
% 
% %% 
% plotangles(Hist.times, Hist.poses);
% 
% %%
% % stateDeriv
% plotvelocity(Hist.times, Hist.twists);
% 
% %%
% plotcontrolforcetorque(Hist.times, Hist.controls);
% 
%%
plotcontrolrpm(Hist.times, Hist.controls);
% 
% %%
% ploterrorquaternion(Hist.times, Hist.controls);

%% Visualize simulation.
simvisualization(Hist.times, Hist.states, 'na');


