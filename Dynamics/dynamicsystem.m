function [stateDeriv] = dynamicsystem(t, state, Control)
% Propagates quadrotor dyanimcs using ode45
% Inputs: time, state and control signal 
% Output: derivative of the state

global g m I Jr PROP_POSNS Kt Dt;

if isreal(Control.rpm) == 0
    error('Control rpm is imaginary');
end

if isreal(state) == 0
    error('State is imaginary');
end


stateDeriv = zeros(13,1);
% define rotation matrix
R = quat2rotmat(state(10:13));

% define forces
fGravity = R * [0; 0; -m*g];                 % force of gravity, body frame
fThrust = [0; 0; -Kt*sum(Control.rpm.^2)];   % force of thrust, body frame

% compute moments
Mx = -Kt*PROP_POSNS(2,:)*(Control.rpm.^2) - state(5) * Jr * sum(rpm2rad(Control.rpm));
My =  Kt*PROP_POSNS(1,:)*(Control.rpm.^2) + state(4) * Jr * sum(rpm2rad(Control.rpm));
Mz =        Dt*[-1 1 -1 1]*(Control.rpm.^2) - Jr*[-1 1 -1 1]* rpm2rad(Control.rpmDeriv);

stateDeriv(1:3)   = (fGravity + fThrust  -  m*cross(state(4:6), state(1:3)))/m;
stateDeriv(4:6)   = inv(I)*([Mx; My; Mz] -  cross(state(4:6), I * state(4:6)));
stateDeriv(7:9)   = R' * state(1:3);
stateDeriv(10:13) = -0.5 * quatmultiply( [0; state(4:6)] , state(10:13));

end

