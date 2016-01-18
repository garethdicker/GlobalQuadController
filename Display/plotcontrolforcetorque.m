function [] = plotcontrolforcetorque(t, ctrl)

    a = struct2cell(ctrl);
    b = [a{5,:}];
    plot(t,b);
    
    legend('u1','u2','u3','u4');
    xlabel('Time (s)');
    ylabel('Force / Moment (N or Nm)');
    title('Control Output Thurst and Moments');
    grid on;
    
end