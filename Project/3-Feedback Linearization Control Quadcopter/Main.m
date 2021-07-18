clc;clear;
close all

global dt
global X_desired
X_desired = [];

Path = 'C:\Users\Iman Sharifi\Documents\MATLAB\Nonlinear Control\Project';
addpath(Path)
Parameters;
Simulation_Params;

t  = 0:dt:Tf;
N  = Tf/dt+1;
% x0 = zeros(12,1);
% % x0(7:2:11) = 1;
x  = x0;
X  = [];
x_ddot = zeros(6,1);
Uxy = [0;0];

for i=1:N
    
    [U, Uxy] = FL_Controller(t(i), x, x_ddot, Uxy);
    k1 = NonLinDynamic_Quadcopter(x,         U);
    k2 = NonLinDynamic_Quadcopter(x+dt/2*k1, U);
    k3 = NonLinDynamic_Quadcopter(x+dt/2*k2, U);
    k4 = NonLinDynamic_Quadcopter(x+dt*k3,   U);
    
    x  = x + dt/6*(k1+2*k2+2*k3+k4);    
    x_ddot = NonLinDynamic_Quadcopter(x, U); 
    x_ddot = x_ddot(2:2:end);
    X = [X;x']; 
    
    if isnan(x(1))
        error('Oops! NaN occured. ---')
    end
end
%% Plot Results
figure
% 3D Position
MyPlot(t,X(:,9),'-b');hold on
MyPlot(t,X(:,11),'-r');
MyPlot(t,X(:,7),'-g')
grid on;title('Position Response');legend('X','Y','Z')

figure
% Attitudes
MyPlot(t,rad2deg(X(:,1)),'-b');hold on
MyPlot(t,rad2deg(X(:,3)),'-r');
grid on;title('Roll & Pitch Angles');legend('\phi','\theta')

figure
MyPlot(t,rad2deg(X(:,5)),'-g')
grid on;title('Yaw Angle');legend('\psi')

%% Save Data
filename = [Path '\MatFiles\FLC_Data_Varriable_Trajectory.mat'];
Data = [X ,X_desired, t'];
save(filename, 'Data')