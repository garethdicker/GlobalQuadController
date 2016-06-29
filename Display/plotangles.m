function [] = plotangles(t, pose)

    a = struct2cell(pose);
    b = [a{3,:}];
    plot(t,b(1,:),'Color', [0.5 0.2 0.4]); 
    plot(t,b(2,:),'Color', [0.1 0.7 0.6]); 
%     plot(t,b(3,:),'Color', [0.6 1.0 0.4]); 

    legend('roll','pitch','yaw');
    xlabel('Time (s)');
    ylabel('Euler Angles (rad)');
    title('Euler Angle Attitude');
    grid on;
end