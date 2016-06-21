%% Validation of Freefall Controller with Monte Carlo simulation

% 120 trials were performed for comparison to Monte Carlo simulation

% Set 1: Upper hemisphere, no yaw,        20 trials
% Set 2: Upper hemisphere, with yaw,      20 trials
% Set 3: Lower hemisphere, no yaw,        20 trials
% Set 4: Lower hemisphere, with yaw,      20 trials

% Set 5: T Configuration full hemisphere, 10 trials
% Set 6: Casual natural throws,           10 trials
% Set 7: Rollrate in full hemisphere,     10 trials
% Set 8: Aggressive natural throws,       10 trials

clear all
% close all
figure
hold on

set = 4;

if set > 4
    num_trials = 10;
else
    num_trials = 20;
end

for i = 1:num_trials
    if (set == 1 && i <= 9)
        trial = ['0', num2str(i)];
    elseif (set >= 7)
        trial = num2str(100 + i + (set-7)*10);
    else
        trial = num2str(i+(set-1)*20);
    end
    file = ['d' trial '.csv'];
    if (strcmp(file,'d29.csv') == 0 && strcmp(file,'d117.csv') == 0)
        M = csvread(file,1,0);
        time = (M(:,1) - M(1,1))*0.000001;
        for j = 1:size(M(:,2))
            if (sqrt(M(j,2).^2 + M(j,3).^2 + M(j,4).^2) < 5.0)
                freefall_time = time(j);
                break;
            end
        end
        time = time - freefall_time;
        for j = 1:size(time)
            if ( abs(M(j,23)) < 0.2 && abs(M(j,24)) < 0.2 && ... 
                   abs(M(j,26)) < 0.2 && abs(M(j,27)) < 0.2 && abs(M(j,28)) < 0.2)
                manual_time = time(j);
                break;
            end
        end

%     %% Attitude Quaternion
%         plot(time, M(:,19), time, M(:,20), time,M(:,21), time,M(:,22));
%         axis([-1, 3, -1.5,1.5]);
        
%% Euler Angles
    plot(time, M(:,23),'Color', [0.5 0.2 0.4]); % roll
    plot(time, M(:,24),'Color', [0.1 0.7 0.6]) % pitch
    plot(time, M(:,25),'Color', [0.6 1.0 0.4]); % yaw 
    axis([0, 2, -4, 4])
    legend('Roll','Pitch');%,'Yaw');
%     line([0, 0],[-6*pi,6*pi],'LineStyle','--','Color','red','LineWidth',3)
%     line([manual_time, manual_time],[-3*pi,3*pi],'LineStyle','--','Color','black','LineWidth',1)

    %% Body Rates
%     plot(time, M(:,26),'Color', [0.5 0.2 0.4])
%     plot(time, M(:,27),'Color', [0.1 0.7 0.6])
%     plot(time, M(:,28),'Color', [0.6 1.0 0.4]);
% 
% %   plot(time, sqrt(M(:,26).^2+M(:,27).^2),'Color', [0.5 0.2 0.4]);
%     axis([0, 2, -10, 15])
%     legend('p','q','r');
% %     line([0, 0],[-6*pi,6*pi],'LineStyle','--','Color','red','LineWidth',3)

%     line([manual_time, manual_time],[-3*pi,3*pi],'LineStyle','--','Color','black','LineWidth',1)

    %% Accelerometer
%         plot(time, sqrt(M(:,2).^2 + M(:,3).^2 + M(:,4).^2));
%         line([-0.5, 1.5],[5,5],'LineStyle','--','Color','red','LineWidth',1)
%         axis([-0.5, 1.5,-10,30]);
    
%% Control Commands
%     plot(time,M(:,32),'Color',[0.4 0.2 1]);
%     plot(time,M(:,33),'Color',[0.4 0.5 0.9]);
%     plot(time,M(:,34),'Color',[0.4 1 0.7]);

%% Thrust Command and PWM Signal
%     plot(time,M(:,35),'Color',[1 0.5 1]);
%     plot(time, sqrt(M(:,36).^2+M(:,37).^2+ M(:,38).^2+M(:,39).^2)/2000,'Color',[0 0.5 1]);
%     legend('Normalized Thrust Command', 'Sum of PWM /2000');

    end
end


% 

%     
%     
% %     line([freefall_time, freefall_time],[-3*pi, 3*pi],'LineStyle','--','Color','red','LineWidth',2)
% line([manual_time, manual_time],[-3*pi,3*pi],'LineStyle','--','Color','green','LineWidth',2)
% 
%     %%
% line([freefall_time, freefall_time],[-3*pi, 3*pi],'LineStyle','--','Color','red','LineWidth',2)
% line([manual_time, manual_time],[-3*pi,3*pi],'LineStyle','--','Color','green','LineWidth',2)

