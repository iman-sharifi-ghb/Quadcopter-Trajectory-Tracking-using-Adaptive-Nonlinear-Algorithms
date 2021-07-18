clc;
clear; close all

% load SMC_Data.mat
% load FLC_Data.mat
% load BSC_Data.mat
% load PID_Data.mat
load LQR_Data.mat

X = Data;
t = X(:,end);

%% Plot Results
figure
% 3D Position
MyPlot(t,X(:,9),'-b');hold on
MyPlot(t,X(:,11),'-r');
MyPlot(t,X(:,7),'-g')
grid on;title('Position Response');legend('X','Y','Z')
xlabel('Time'), ylabel('Amp')

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