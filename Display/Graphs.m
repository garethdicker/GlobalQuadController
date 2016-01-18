function [  ] = Graphs( ttotal,Xtotal,roll_hist,pitch_hist,pdes_hist, qdes_hist, quat_hist,u1,u2,u3,u4)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

figure('units','normalized','outerposition',[0 0 1 1])
% figure();
% subplot(2,4,1);
% plot(ttotal,Xtotal(:,1),ttotal,Xtotal(:,2),ttotal,Xtotal(:,3));
% legend('u','v','w','Location','southoutside','Orientation','horizontal');
% xlabel('Time (s)');
% ylabel('Velocity (m/s)');
% title('Body linear velocities');
% grid on;
% ax = gca;
% axis([ttotal(1) ttotal(end) ax.YLim]);

% subplot(2,4,2);
% plot(ttotal,Xtotal(:,4),ttotal,Xtotal(:,5),ttotal,Xtotal(:,6));
% legend('p','q','r','Location','southoutside','Orientation','horizontal');
% xlabel('Time (s)');
% ylabel('Angular Rate (rad/s)');
% title('Body angular velocities');
% grid on;
% ax = gca;
% axis([ttotal(1) ttotal(end) ax.YLim]);

% subplot(2,4,3);
% ax=gca;
% plot(ttotal,Xtotal(:,7),ttotal,Xtotal(:,8),ttotal,Xtotal(:,9));
% legend('X^w','Y^w','Z^w','Location','southoutside','Orientation','horizontal');
% % ax.XTick = [0 1 2 3 4 5 6 7 8 9 10];
% xlabel('Time (s)');
% ylabel('World Position (m)');
% title('World positions');
% grid on;
% ax = gca;
% axis([ttotal(1) ttotal(end) ax.YLim]);

% subplot(2,3,4);
% plot(ttotal,Xtotal(:,10),ttotal,Xtotal(:,11),ttotal,Xtotal(:,12),ttotal,Xtotal(:,13));
% legend('q_0','q_1','q_2','q_3');
% xlabel('Time (s)');
% grid on;
% ylabel('');

% subplot(2,4,4)
% plot(ttotal,quat_hist);
% legend('quat','Location','southoutside','Orientation','horizontal');
% xlabel('Time (s)');
% ylabel('Components (rad');
% title('Desired vs. Estimated Quaternion');
% grid on;
% ax = gca;
% axis([ttotal(1) ttotal(end) ax.YLim]);

subplot(2,2,1)
plot(ttotal,pdes_hist,ttotal,Xtotal(:,4));
legend('p_{des}','p','Location','southoutside','Orientation','horizontal');
xlabel('Time (s)');
ylabel('Angular rate (rad) /sec');
title('Desired vs. Actual "p" Body Rate');
grid on;
ax = gca;
% axis([ttotal(1) ttotal(end) ax.YLim]);

subplot(2,2,2)
plot(ttotal,qdes_hist,ttotal,Xtotal(:,5));
legend('q_{des}','q','Location','southoutside','Orientation','horizontal');
xlabel('Time (s)');
ylabel('Angular rate (rad) / sec');
title('Desired vs. Actual "q" Body Rate');
grid on;
ax = gca;
% axis([ttotal(1) ttotal(end) ax.YLim]);

% subplot(2,4,7)
% plot(ttotal,rdes_hist,ttotal,Xtotal(:,6));
% legend('r_{des}','r','Location','southoutside','Orientation','horizontal');
% xlabel('Time (s)');
% ylabel('Angle (rad)');
% title('Des. & Act. r');
% grid on;
% ax = gca;
% axis([ttotal(1) ttotal(end) ax.YLim]);

subplot(2,2,3)
plot(ttotal,-u1,ttotal,u2,ttotal,u3,ttotal,u4);
legend('U1','U2','U3','U4','Location','southoutside','Orientation','horizontal');
xlabel('Time (s)');
ylabel('Thrust (N)/ Moment (Nm)');
title('F & T Signals');
grid on;
ax = gca;
% axis([ttotal(1) ttotal(end) ax.YLim]);

% 
subplot(2,2,4);
plot(ttotal,roll_hist,ttotal,pitch_hist);
legend('roll_des','pitch_des');
xlabel('Time (s)');
ylabel('Angles (rad)');
grid on;
% yl = ylim;

% subplot(2,3,6);
% plot(ttotal,roll_hist,ttotal,pitch_hist,ttotal,yaw_hist);
% legend('roll','pitch','yaw');
% xlabel('Time (s)');
% ylabel('Angle (rad)');
% % ylim(yl);
% grid on;

end

