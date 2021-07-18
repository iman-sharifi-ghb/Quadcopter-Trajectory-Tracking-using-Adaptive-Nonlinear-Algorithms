clc;clear;
close all
global dt
global A B C D desiredPoles 
global Q R N
global Am Bm

Path = 'C:\Users\Iman Sharifi\Documents\MATLAB\Nonlinear Control\Project';
addpath(Path)
Simulation_Params;
Parameters;

% Linearization
[A,B,C,D]=State_Space();
n = length(A);
ud = size(B, 2);

% State Feedback params
desiredPoles = -linspace(1,n,n);
% LQR params
Q = 10*eye(n);
Q(7,7) = 1000000;
Q(8,8) = 0;
Q(5,5) = 20;
R = 5*eye(ud);
N = 0;

Type = 'lqr'; % or 'place'
K = SF_Controller(Type);

KI = B\(A-B*K);
ud  = zeros(12,1);ud(7:2:11) = 1;

% Initialization Main Loop
dt = 0.005;
Tf = 60;
t  = 0:dt:Tf;
N  = Tf/dt+1;
x0 = zeros(12,1);
% x0(1:2:5) = deg2rad(1);
% x0(7:2:11) = 5;
x  = x0;
xd = x0;
X  = [];
Xd  = [];
x_ddot = zeros(6,1);
Uxy = [0;0];

%% Reference Model params
Am = A - B*K;
Bm = B;
Q = 600*eye(12); 
P = lyap(Am',Q);

% Learning Rates
gamma_x = 0.01*eye(12);  
gamma_r = 0.0005*eye(4);     

% Adaption gains initialization
Kx = -K;
Kr = eye(4);

%% Desired Trajectories
x_ref =[repelem([1 2 1 0 1 2], floor(N/6)) 2];
y_ref =[repelem([1 2 1 0 1 2], floor(N/6)) 2];
z_ref =[repelem([1 2 1 0 1 2], floor(N/6)) 2]+1;

ref = repmat(zeros(12,1), [1, N]);
ref(9,:) = x_ref;
ref(11,:)= y_ref;
ref(7,:) = z_ref;

%% Main Loop
for i=1:N
    
    r = -KI*ref(:,i);
    u  = Kx*x + Kr'*r; 
    
    x  = RK4(@NonLinDynamic_Quadcopter, x, u);
    
    ud = - KI*ref(:,i);
    xd = RK4(@ReferenceModel, xd, ud);
    
    e = x - xd;
    disp(['Error X is: ' num2str(e(9))])
    
    Kx = Kx - (gamma_x * x * e' * P * B)'*dt ;
    Kr = Kr - (gamma_r * r * e' * P * B)'*dt ;

    X = [X; x'];  
    Xd= [Xd; xd'];
    
end
%% Plot Results
% 3D Position
figure
MyPlot(t,X(:,9 ),'-b');hold on
MyPlot(t,Xd(:,9),'-r')
MyPlot(t,ref(9,:),'-g')
grid on;title('X');legend('X','X desired','X reference')
xlabel('Time'), ylabel('Amp')
figure
MyPlot(t,X(:,11),'-b');hold on
MyPlot(t,Xd(:,11),'-r')
MyPlot(t,ref(11,:),'-g')
grid on;title('Y');legend('Y','Y desired','Y reference')
xlabel('Time'), ylabel('Amp')
figure
MyPlot(t,X(:,7),'-b');hold on
MyPlot(t,Xd(:,7),'-r')
MyPlot(t,ref(7,:),'-g')
grid on;title('Z');legend('Z','Z desired','Z reference')
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
%% Save Data
filename = [Path '\MatFiles\MRAC_Data.mat'];
Data = [X,Xd,ref',t'];
save(filename, 'Data')