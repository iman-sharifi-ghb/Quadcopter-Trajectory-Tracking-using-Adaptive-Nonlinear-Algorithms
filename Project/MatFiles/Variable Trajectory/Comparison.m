clc;
clear; close all

load SMC_Data_Varriable_Trajectory.mat
X4 = Data;
load FLC_Data_Varriable_Trajectory.mat
X3 = Data;
load BSC_Data_Varriable_Trajectory.mat
X5 = Data;
% load PID_Data.mat
% X1 = Data;
% load LQR_Data.mat
% X2 = Data;

t = X3(:,end);

%% Plot Results
figure
plot(t,X3(:,9 ),'-b','LineWidth', 3);hold on
plot(t,X4(:,9 ),'-r','LineWidth', 3);hold on
plot(t,X5(:,9 ),'-y','LineWidth', 3);
plot(t,X3(:,17 ),'--g','LineWidth', 3);

grid on;title('X');legend('FLC','SMC','BSC','Reference')
xlabel('Time'), ylabel('Amp')
set(gca, 'FontSize', 14)
figure
plot(t,X3(:,11 ),'-b','LineWidth', 3);hold on
plot(t,X4(:,11 ),'-r','LineWidth', 3);hold on
plot(t,X5(:,11 ),'-y','LineWidth', 3);
plot(t,X3(:,18 ),'--g','LineWidth', 3);
grid on;title('Y');legend('FLC','SMC','BSC','Reference')
xlabel('Time'), ylabel('Amp')
set(gca, 'FontSize', 14)
figure
plot(t,X3(:,7 ),'-b','LineWidth', 3);hold on
plot(t,X4(:,7 ),'-r','LineWidth', 3);hold on
plot(t,X5(:,7 ),'-y','LineWidth', 3);
plot(t,X3(:,16 ),'--g','LineWidth', 3);
grid on;title('Z');legend('FLC','SMC','BSC','Reference')
xlabel('Time'), ylabel('Amp')
set(gca, 'FontSize', 14)

% figure
% % Attitudes
% plot(t,rad2deg(X(:,1)),'-b','LineWidth', 3);hold on
% plot(t,rad2deg(X(:,3)),'-r','LineWidth', 3);hold on
% plot(t,rad2deg(X(:,5)),'-g','LineWidth', 3)
% grid on;title('Euler Angles');legend('\phi','\theta','\psi')
% xlabel('Time'), ylabel('Amp')