clc;
clear; close all

load SMC_Data.mat
X4 = Data;
load FLC_Data.mat
X3 = Data;
load BSC_Data.mat
X5 = Data;
load PID_Data.mat
X1 = Data;
load LQR_Data.mat
X2 = Data;

t = X1(:,end);

%% Plot Results
figure
MyPlot(t,X1(:,9 ),'-b');hold on
MyPlot(t,X2(:,9 ),'-r');hold on
MyPlot(t,X3(:,9 ),'-g');hold on
MyPlot(t,X4(:,9 ),'-y');hold on
MyPlot(t,X5(:,9 ),'-p');
grid on;title('X');legend('PID','LQR','FLC','SMC','BSC')
xlabel('Time'), ylabel('Amp')
figure
MyPlot(t,X1(:,11 ),'-b');hold on
MyPlot(t,X2(:,11 ),'-r');hold on
MyPlot(t,X3(:,11 ),'-g');hold on
MyPlot(t,X4(:,11 ),'-y');hold on
MyPlot(t,X5(:,11 ),'-p');
grid on;title('Y');legend('PID','LQR','FLC','SMC','BSC')
xlabel('Time'), ylabel('Amp')
figure
MyPlot(t,X1(:,7 ),'-b');hold on
MyPlot(t,X2(:,7 ),'-r');hold on
MyPlot(t,X3(:,7 ),'-g');hold on
MyPlot(t,X4(:,7 ),'-y');hold on
MyPlot(t,X5(:,7 ),'-p');
grid on;title('Z');legend('PID','LQR','FLC','SMC','BSC')
xlabel('Time'), ylabel('Amp')

% figure
% % Attitudes
% MyPlot(t,rad2deg(X(:,1)),'-b');hold on
% MyPlot(t,rad2deg(X(:,3)),'-r');hold on
% MyPlot(t,rad2deg(X(:,5)),'-g')
% grid on;title('Euler Angles');legend('\phi','\theta','\psi')
% xlabel('Time'), ylabel('Amp')