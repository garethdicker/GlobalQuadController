
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
global pPos dPos;

% Initialize global parameters.
initparams;

% Define starting pose and twist parameters.
IC.posn     = [0; 0; 2]; % world frame position (meters) 
IC.linVel   = [0; 0; 0]; % world frame velocity (m / s)
IC.angVel   = [0; 0; 0]; % body rates (radians / s)
IC.attEuler = [pi/2; 0; 0]; % [roll; pitch; yaw] (radians)

% Initialize state and its derivative.
[state, stateDeriv] = initstate(IC);

endTime = 2;  % seconds
dt = 1 / 200; % time step (Hz)

% Define pose and twist structs.
[Pose, Twist] = defineposetwist;

% Define control types using pose and twist struct types.
Control = definecontrol(Pose, Twist);

% Update pose and twist structs from state and its derivative.
[Pose, Twist] = updatekinematics(state, stateDeriv);

% Initialize history for plotting and visualizing simulation.
Hist = inithistory(state, stateDeriv, Twist, Pose, Control);

%% Simulation
for i = 0 : dt : endTime - dt
    
    % Set control input for recovery controller.
    % TODO: make world frame not body
    Control.acc = [0; 0; 9.81];
    
    [recoveryStage] = checkrecoverystage(Pose, Twist);
    
    [Control] = computedesiredacceleration(Control, Pose, Twist, recoveryStage);
    
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
    Hist = updatehistory(Hist, t, state, stateDeriv);

    % TODO: put back into updatehistory function where they don't work
    Hist.poses = [Hist.poses; Pose];
    Hist.twists = [Hist.twists; Twist];
    Hist.controls = [Hist.controls; Control];

end
%% Display Plots
plotbodyrates(Hist.t, Hist.controls, Hist.twists);

%% TODO: check actual acceleration values
plotaccelerations(Hist.t, Hist.controls, Hist.stateDerivs(1:3,:), Hist.states(10:13,:));

%% 
plotposition(Hist.t, Hist.poses);

%% 
plotangles(Hist.t, Hist.poses);

%%
plotvelocity(Hist.t, Hist.twists);

%%
% plotcontrolforcetorque(Hist.t, Hist.controls);

%%
plotcontrolrpm(Hist.t, Hist.controls);

%%
% ploterrorquaternion(Hist.t, Hist.controls);

%% Visualize simulation.
simvisualization(Hist.t, Hist.states, 'na');


