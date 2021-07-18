clc;clear;
close all

global dt
global S S_dot X_desired
S = [];S_dot = [];X_desired = [];

Path = 'C:\Users\Iman Sharifi\Documents\MATLAB\Nonlinear Control\Project';
addpath(Path)
Parameters;
Simulation_Params;

t  = 0:dt:Tf;
N  = Tf/dt+1;
x  = x0;
X  = [];
x_ddot = zeros(6,1);
Uxy = [0;0];

for i=1:N
    
    [U, Uxy] = SMC_Controller(t(i), x, x_ddot, Uxy);
    k1 = NonLinDynamic_Quadcopter(x,         U);
    k2 = NonLinDynamic_Quadcopter(x+dt/2*k1, U);
    k3 = NonLinDynamic_Quadcopter(x+dt/2*k2, U);
    k4 = NonLinDynamic_Quadcopter(x+dt*k3,   U);
    
    x  = x + dt/6*(k1+2*k2+2*k3+k4);    
    x_ddot = NonLinDynamic_Quadcopter(x, U); 
    x_ddot = x_ddot(2:2:end);
    X = [X;x'];    
end
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
%% Sliding Surfaces
figure
title('Sliding Surafaces');
% 3D Position
subplot(221); MyPlot(S(:,1),S_dot(:,1));
grid on; xlabel('S_1'), ylabel('DS_1')
subplot(222); MyPlot(S(:,2),S_dot(:,2));
grid on; xlabel('S_2'), ylabel('DS_2')
subplot(223); MyPlot(S(:,3),S_dot(:,3));
grid on; xlabel('S_3'), ylabel('DS_3')
subplot(224); MyPlot(S(:,4),S_dot(:,4));
grid on; xlabel('S_4'), ylabel('DS_4')

%% Video Animation
makeVideo = input('Would you like to show Animation???');
if makeVideo==1
    close all
    tic;
    figure;
    Xd = X_desired(:,5);
    Yd = X_desired(:,6);
    Zd = X_desired(:,4);
    Quad_Animation(t,X(:,9),X(:,11),X(:,7),X(:,1),X(:,3),X(:,5),Xd,Yd,Zd)
    toc
end

%% Save Data
% filename = [Path '\MatFiles\SMC_Data_Varriable_trajectory.mat'];
% Data = [X, X_desired, t'];
% save(filename, 'Data')