function [U, Uxy] = FL_Controller(t, x, y, Uxy)

global X_desired

%% Actual States
X = x(9) ;X_dot = x(10);X_ddot = y(5);
Y = x(11);Y_dot = x(12);Y_ddot = y(6);
Z = x(7) ;Z_dot = x(8);Z_ddot = y(4);

Phi = x(1);Phi_dot = x(2);Phi_ddot = y(1);
Tta = x(3);Tta_dot = x(4);Tta_ddot = y(2);
Psi = x(5);Psi_dot = x(6);Psi_ddot = y(3);

%% Desired States
[DesPos, DesAtt] = CreateDesiredTrajectory(t);

Xd = DesPos(4);
Yd = DesPos(7);
Zd = DesPos(1);

Xd_dot = DesPos(5);
Yd_dot = DesPos(8);
Zd_dot = DesPos(2);

Xd_ddot = DesPos(6);
Yd_ddot = DesPos(9);
Zd_ddot = DesPos(3);

Phid = DesAtt(1);
Ttad = DesAtt(4);
Psid = DesAtt(7);

Phid_dot = DesAtt(2);
Ttad_dot = DesAtt(5);
Psid_dot = DesAtt(8);

Phid_ddot = DesAtt(3);
Ttad_ddot = DesAtt(6);
Psid_ddot = DesAtt(9);

X_desired = [X_desired;[Phid,Ttad,Psid,Zd,Xd,Yd]];

%% Parameter Definition
global m g dt
global a1 a2 a3
global b1 b2 b3
global K1 K2 %K3 K4 K5 K6 K7 K8 K9 K10 K11 K12
global KI
global int_e_Z   int_e_X   int_e_Y
global int_e_Phi int_e_Tta int_e_Psi

%% Altitude Control
e4     = Zd - Z;
e4_dot = Zd_dot - Z_dot;
int_e_Z = int_e_Z + e4;

v1 = [K1 K2]*[e4;e4_dot]+Zd_ddot;%+KI*int_e_Z;

U1 = m/cos(Phi)/cos(Tta)*(g+v1);
%% x Motion Control
e5     = Xd - X;
e5_dot = Xd_dot - X_dot;
disp(['Error x: ' num2str(e5)])
int_e_X = int_e_X + e5;

vx = [K1 K2]*[e5;e5_dot]+Xd_ddot;%+KI*int_e_X;

Ux     = m/U1*vx;
Ux_dot = (Ux - Uxy(1))/dt;
%% y Motion Control
e6     = Yd - Y;
e6_dot = Yd_dot - Y_dot;
int_e_Y = int_e_Y + e6;

vy = [K1 K2]*[e6;e6_dot]+Yd_ddot;%+KI*int_e_Y;

Uy     = m/U1*vy;
Uy_dot = (Uy - Uxy(2))/dt;
%% Roll Control
e1     = -Uy - Phi;
e1_dot = -Uy_dot - Phi_dot;
int_e_Phi = int_e_Phi + e1;

v2 = [K1 K2]*[e1;e1_dot];%+KI*int_e_Phi;
U2 = 1/b1*(-a1*Tta_dot*Psi_dot+v2);
%% Pitch Control
e2     = Ux - Tta;
e2_dot = Ux_dot - Tta_dot;
int_e_Tta = int_e_Tta + e2;

v3 = [K1 K2]*[e2;e2_dot];%+KI*int_e_Tta;
U3 = 1/b2*(-a2*Phi_dot*Psi_dot+v3);
%% Yaw Control
e3      = Psid - Psi;
e3_dot  = Psid_dot - Psi_dot;
int_e_Psi = int_e_Psi + e3;

v4 = [K1 K2]*[e3;e3_dot]+Psid_ddot;%+KI*int_e_Psi;
% disp([num2str(Psid) ' and ' num2str(Psi)])

U4 = 1/b3*(-a3*Phi_dot*Tta_dot+v4);
%2- PID: K1*e3+K2*e3_dot+K3*e3_ddot;K1 = 0;K2 = 0;K3 = 1;
%3- K51*s3/(abs(s3)+delta)+;delta = 0.3;
%% Control Inputs
U = [U1;U2;U3;U4];
Uxy = [Ux;Uy];

end

%% 
function out = SignApprox(S, Type, a)

if nargin<2
    Type = 1;
end

switch Type
    case 1
        out = sign(S);
    case 2
        phi = a;
        out = Saturation(S/phi, [-1 1]);
    case 3
        coef = a;
        out = tanh(coef*S);
end

end
function out = Saturation(S, bound)

if S > max(bound)
    out = max(bound);
elseif S < min(bound)
    out = min(bound);
else
    out = S;
end

end
%%

