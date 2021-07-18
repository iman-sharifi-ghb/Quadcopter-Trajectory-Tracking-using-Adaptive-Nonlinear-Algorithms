clc;
clear; close all

load SMC_Data_Varriable_Trajectory.mat
% load FLC_Data_Varriable_Trajectory.mat
% load BSC_Data_Varriable_Trajectory.mat
% load PID_Data_Varriable_Trajectory.mat
% load LQR_Data_Varriable_Trajectory.mat

X = Data;
t = X(:,end);

%% Plot Results
figure
% 3D Position
plot(t,X(:,9),'-b','LineWidth', 3);hold on
plot(t,X(:,17),'--g','LineWidth', 3)
grid on;title('X');legend('X','X desired')
xlabel('Time'), ylabel('Amp')
set(gca, 'FontSize', 14)

figure
plot(t,X(:,9),'-b','LineWidth', 3);hold on
plot(t,X(:,17),'--g','LineWidth', 3)
grid on;title('X');legend('X','X desired')
xlabel('Time'), ylabel('Amp')
set(gca, 'FontSize', 14)

figure
plot(t,X(:,7),'-b','LineWidth', 3);hold on
plot(t,X(:,16),'--g','LineWidth', 3)
grid on;title('Z');legend('Z','Z desired')
xlabel('Time'), ylabel('Amp')
set(gca, 'FontSize', 14)

figure
% Attitudes
MyPlot(t,rad2deg(X(:,1)),'-b');hold on
MyPlot(t,rad2deg(X(:,3)),'-r');hold on
MyPlot(t,rad2deg(X(:,5)),'-g')
grid on;title('Euler Angles');legend('\phi','\theta','\psi')
xlabel('Time'), ylabel('Amp')

% figure
% MyPlot(t,rad2deg(X(:,5)),'-g')
% grid on;title('Yaw Angle');legend('\psi')
% xlabel('Time'), ylabel('Amp')