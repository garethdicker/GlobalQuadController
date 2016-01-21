function [] = ploterrorquaternion(t, ctrl)

    hold on
    % desired
    a = struct2cell(ctrl);
    b = [a{4,:}];
    plot(t,b);

    legend('q0','q1','q2','q3');
    xlabel('Time (s)');
    ylabel('Unit Quaternion Values');
    title('Error Quaternion');
    grid on;