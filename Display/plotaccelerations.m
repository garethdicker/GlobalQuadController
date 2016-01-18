function [] = plotaccelerations(t, ctrl, bodyAccelerations, quatHist)

    hold on
%     % desired accleration
%     controlCells = struct2cell(ctrl);
%     desiredAccelerations = [controlCells{3,:}];
%     plot(t,desiredAccelerations);
    
    % actual acceleration (world frame conversion)
    worldAccelerations = zeros(size(bodyAccelerations));
    for i = 1:length(t)
        rotate = quat2rotmat(quatHist(:,i));
        worldAccelerations(:,i) = rotate'*bodyAccelerations(:,i);
    end
    plot(t,worldAccelerations);
    
    legend('accXDes','accYDes','accZDes');%,'accX','accY','accZ');
    xlabel('Time (s)');
    ylabel('Accelerations (m)');
    title('Desired vs. Actual Acceleration');
    grid on;
end