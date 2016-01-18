function [] = plotbodyrates(t, ctrl, twists)

    hold on
    % desired
    a = struct2cell(ctrl);
    b = [a{2,:}];
    c = [b.angVel];
    plot(t,real(c(1,:)),'--r');
    plot(t,real(c(2,:)),'--g');
    plot(t,real(c(3,:)),'--b');
    
    %actual
    a = struct2cell(twists);
    b = [a{3,:}];
    plot(t,real(b(1,:)),'r');
    plot(t,real(b(2,:)),'g');
    plot(t,real(b(3,:)),'b');
    
    legend('pDes','qDes','rDes','p','q','r');
    xlabel('Time (s)');
    ylabel('Body rate (rad/s)');
    title('Desired vs. Actual Body Rates');
    grid on;
end