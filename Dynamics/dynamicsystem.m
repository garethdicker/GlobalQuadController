function [stateDeriv, PropState] = dynamicsystem(t, state, dt, rpmControl, rpmPrev)
% Propagates quadrotor dyanimcs using ode45
% Inputs: time, state and control signal 
% Output: derivative of the state

global g m I Jr PROP_POSNS Kt Dt;

if isreal(rpmControl) == 0
    error('Control rpm is imaginary');
end

if isreal(state) == 0
    error('State is imaginary');
end

rpmDeriv = (rpm2rad(rpmControl) - rpm2rad(rpmPrev))/dt;


% seems like 40,000 rpm/sec is the saturation limit
rpmMaxAccel = 30000;

for i = 1:4
    if (abs(rpmDeriv(i)) > rpmMaxAccel)
        rpmDeriv(i) = sign(rpmDeriv(i))*rpmMaxAccel;
        rpmControl(i) = rpmPrev(i) + rpmDeriv(i)*dt;
    end
end

PropState.rpm = rpmControl;
PropState.rpmDeriv = rpmDeriv;
    
stateDeriv = zeros(13,1);
% define rotation matrix
R = quat2rotmat(state(10:13));

% define forces
fGravity = R * [0; 0; -m*g];                 % force of gravity, body frame
fThrust = [0; 0; -Kt*sum(rpmControl.^2)];   % force of thrust, body frame

% compute moments
Mx = -Kt*PROP_POSNS(2,:)*(rpmControl.^2) - state(5) * Jr * sum(rpm2rad(rpmControl));
My =  Kt*PROP_POSNS(1,:)*(rpmControl.^2) + state(4) * Jr * sum(rpm2rad(rpmControl));
Mz =        Dt*[-1 1 -1 1]*(rpmControl.^2) - Jr*[-1 1 -1 1]* rpm2rad(rpmDeriv);

stateDeriv(1:3)   = (fGravity + fThrust  -  m*cross(state(4:6), state(1:3)))/m;
stateDeriv(4:6)   = inv(I)*([Mx; My; Mz] -  cross(state(4:6), I * state(4:6)));
stateDeriv(7:9)   = R' * state(1:3);
stateDeriv(10:13) = -0.5 * quatmultiply( [0; state(4:6)] , state(10:13));

end

