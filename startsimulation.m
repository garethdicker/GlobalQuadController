
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

endTime = 3;  % seconds
dt = 1 / 200; % time step (Hz)
    
iterations = 10000;

% count how many of the iterations stabilizes
recoveryCount = 0;

% count the times at which stabilization occured
recoveryTime = 0;

% measure how far from the origin the quadrotor has drifted
maxPosition = [0 0 0];

for k = 1:iterations
    display(k);
    % Define starting pose and twist parameters.
    IC.posn     = [0; 0; 0]; % world frame position (meters) 
    IC.linVel   = [54*(rand-0.5); 54*(rand-0.5); 10*(rand-0.5)]; % world frame velocity (m / s)
    
    % bounds on initial angular velocities taken from maximum expected
    % angular rates during experiments
    IC.angVel   = [54*(rand-0.5); 54*(rand-0.5); 10*(rand-0.5)]; % body rates (radians / s)
    
    % bounds on initial roll and pitch allow from upside down starting
    % configuration
    IC.attEuler = [2*pi*(rand-0.5); 2*pi*(rand-0.5); 0]; % [roll; pitch; yaw] (radians)
    
    % Initialize state and its derivative.
    [state, stateDeriv] = initstate(IC);

    % Define control types using pose and twist struct types.
    Control = initcontrol;

    % Initialize propeller state
    PropState = initpropstate([0; 0; 0; 0]);

    % Update pose and twist structs from state and its derivative.
    [Pose, Twist] = updatekinematics(state, stateDeriv);

    % Initialize history for plotting and visualizing simulation.
    Hist = inithist(state, stateDeriv, Pose, Twist, Control);
    
    k
    prevStage = 1;
    recoveryStage = 1;

    % Simulation
    for i = 0 : dt : endTime - dt
        
        [recoveryStage] = checkrecoverystage(Pose, Twist, recoveryStage);
        if recoveryStage == 4 && prevStage ~= 4
            tempRecoveryTime = i;
        end
        prevStage = recoveryStage;

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
    
        if recoveryStage == 4
            recoveryCount = recoveryCount + 1;
            recoveryTime = [recoveryTime; tempRecoveryTime];
        end
        maxPosition = [maxPosition; Pose.posn'];
end
%
recoveryTime = recoveryTime(2:end);
%% Create histogram of recovery times
figure
hist(recoveryTime,10);

%% Create histogram of maximum positions
figure
hist(maxPosition(maxPosition(:,3)~=0,3),10);


%% Display Plots
% plotbodyrates(Hist.times, Hist.controls, Hist.twists);

%% TODO: change
% plotaccelerations(Hist.times, Hist.controls, Hist.twists);
% 
%% 
plotposition(Hist.times, Hist.poses);
% 
%% 
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
% plotcontrolrpm(Hist.times, Hist.controls);
% 
% %%
% ploterrorquaternion(Hist.times, Hist.controls);

%% Visualize simulation.
simvisualization(Hist.times, Hist.states, 'YZ');


