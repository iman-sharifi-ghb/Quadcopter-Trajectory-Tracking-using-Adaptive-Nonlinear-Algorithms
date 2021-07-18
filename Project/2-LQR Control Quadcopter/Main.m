clc;clear;
close all
global dt
global A B C D desiredPoles 
global Q R N

Path = 'C:\Users\Iman Sharifi\Documents\MATLAB\Nonlinear Control\Project';
addpath(Path)
Simulation_Params;
Parameters;

[A,B,C,D]=State_Space();
n = length(A);
r = size(B, 2);

% State Feedback params
start = 3;step = 0.1;
desiredPoles = [-1:-1:-12];%-start:-step:-(start+step*(n-1));
desiredPoles(5) = -20;
desiredPoles(6) = -30;
% LQR params
Q = 10*eye(n);
Q(7,7) = 1000000;
Q(8,8) = 0;
Q(5,5) = 20;
R = 5*eye(r);
N = 0;

K = SF_Controller('place');

KI = B\(A-B*K);
r  = zeros(12,1);r(7:2:11) = 1;
r(5) = deg2rad(1);

% dt = 0.005;
% Tf = 50;
t  = 0:dt:Tf;
N  = Tf/dt+1;
% x0 = zeros(12,1);
% x0(5) = deg2rad(1);
% x0(7:2:11) = 1;
x  = x0;
X  = [];
x_ddot = zeros(6,1);
Uxy = [0;0];

for i=1:N
    
    [DesPos, DesAtt] = CreateDesiredTrajectory(0);
    r = [DesAtt([1:2,4:5,7:8]); DesPos([1:2,4:5,7:8])];
    U = -K*x - KI*r;
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

figure
% Attitudes
MyPlot(t,rad2deg(X(:,1)),'-b');hold on
MyPlot(t,rad2deg(X(:,3)),'-r');
grid on;title('Roll & Pitch Angles');legend('\phi','\theta')

figure
MyPlot(t,rad2deg(X(:,5)),'-g')
grid on;title('Yaw Angle');legend('\psi')
%% Save Data
filename = [Path '\MatFiles\LQR_Data.mat'];
Data = [X,t'];
save(filename, 'Data')