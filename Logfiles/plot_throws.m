%% Plot Quaternion
clear all
hold on
for i = 1:1
    M = csvread('d119.csv',1,0);
    time = (M(:,1) - M(1,1))*0.000001;
    plot(time, M(:,19), time, M(:,20), time,M(:,21), time,M(:,22));
%     plot(time, sqrt(M(:,2).^2 + M(:,3).^2 + M(:,4).^2));

end
plot(time, M(:,36)/1000, time,M(:,37)/1000, time,M(:,38)/1000, time,M(:,39)/1000);
