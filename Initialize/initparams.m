function []= initparams()

    global g m I Jr propLocation Kt Dt Ixx Iyy Izz u2RpmMat pPos dPos pXY dXY pZ dZ;

    g = 9.81;   % gravity
    m = 0.933;  % mass (kg) 

    % inertial properties in body frame
    Ixx = 0.008737;
    Iyy = 0.008988;
    Izz = 0.017143;
    
    Ixy = -4.2e-7;
    Iyz = -1.14e-6;
    Izx = -5.289e-5;

    I = [Ixx Ixy Izx;Ixy Iyy Iyz;Izx Iyz Izz]; 

    Jr = 2.20751e-5; %Propeller moment of inertia about rotation axis (kg m^2)

    % stores geometry of Spiri
    load('locations2');
    
    %prop locations relative to CoM
    propLocation = [p1, p2, p3, p4] - repmat(CoM,1,4);

    Kt = 7.015e-8; % Thrust coefficient
    Dt = 9.61e-10; % Drag coefficient
    
    u2RpmMat = inv([-Kt                   -Kt                   -Kt                   -Kt;              
                    -Kt*propLocation(2,1) -Kt*propLocation(2,2) -Kt*propLocation(2,3) -Kt*propLocation(2,4);
                     Kt*propLocation(1,1)  Kt*propLocation(1,2)  Kt*propLocation(1,3)  Kt*propLocation(1,4);
                    -Dt                    Dt                   -Dt                    Dt                 ]);
    
    % position and velocity proportional gain matricies
    pPos = zeros(3,3);
    dPos = zeros(3,3);

end

