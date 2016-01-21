function [] = plotaccelerations(t, ctrl, bodyAccelerations, quatHist)

    hold on
%     % desired accleration
    controlCells = struct2cell(ctrl);
    desiredAccelerations = real([controlCells{3,:}]);
    plot(t,desiredAccelerations(1,:),'--r');
    plot(t,desiredAccelerations(2,:),'--g');
    plot(t,desiredAccelerations(3,:)-9.81,'--b');

    
    % actual acceleration (world frame conversion)
    worldAccelerations = zeros(size(bodyAccelerations));
    for i = 1:length(t)
        rotate = quat2rotmat(quatHist(:,i));
        worldAccelerations(:,i) = rotate'*bodyAccelerations(:,i);
    end
    
    plot(t,worldAccelerations(1,:),'r');
    plot(t,worldAccelerations(2,:),'g');
    plot(t,worldAccelerations(3,:),'b');
    
%     plot(t,bodyAccelerations(1,:),'r');
%     plot(t,bodyAccelerations(2,:),'g');
%     plot(t,bodyAccelerations(3,:),'b');
    
    legend('accXDes','accYDes','accZDes','accX','accY','accZ');
    xlabel('Time (s)');
    ylabel('Accelerations (m)');
    title('Desired vs. Actual Acceleration');
    grid on;
end