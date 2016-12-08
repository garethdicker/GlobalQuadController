close all
subplot(3,1,1)
hist(recoveryTime,20)
xlabel('Time until stability (sec)');
ylabel('Number of simulations');
grid on;
subplot(3,1,2)
hist(-maxPosition(:,3),20)
xlabel('Height loss (meters)');
ylabel('Number of simulations');
grid on;
subplot(3,1,3)
hist(sqrt(maxPosition(:,1).^2+maxPosition(:,2).^2),20)
xlabel('Horizontal drift (meters)');
ylabel('Number of simulations');
grid on;