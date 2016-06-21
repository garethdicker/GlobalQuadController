function [] = plotbodyrates(t, ctrl, twists)

    hold on
    % desired
%     a = struct2cell(ctrl);
%     b = [a{2,:}];
%     c = [b.angVel];
%     plot(t,c(1,:),'--r');
%     plot(t,c(2,:),'--g');
%     plot(t,c(3,:),'--b');
    
    %actual
    d = struct2cell(twists);
    e = [d{4,:}];
    plot(t,e(1,:),'Color', [0.5 0.2 0.4]);
    plot(t,e(2,:),'Color', [0.1 0.7 0.6]);
    plot(t,e(3,:),'Color', [0.6 1.0 0.4]);

    
    legend('p','q','r');
    xlabel('Time (s)');
    ylabel('Body rate (rad/s)');
    title('Body Rates');
    grid on;
end