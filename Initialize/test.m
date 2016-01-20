u= [-9.10700447366515;-5.41468322378214;0;0];

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
                
rpm = sqrt(abs(u2RpmMat*u))