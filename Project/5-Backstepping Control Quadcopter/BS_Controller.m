function [U, Uxy] = BS_Controller(t, x, Uxy, Uxy2)

global X_desired

%% Actual States
X = x(9) ;X_dot = x(10);
Y = x(11);Y_dot = x(12);
Z = x(7) ;Z_dot = x(8);

Phi = x(1);Phi_dot = x(2);
Tta = x(3);Tta_dot = x(4);
Psi = x(5);Psi_dot = x(6);

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
global c1 c2 c3 c4  c5  c6
global c7 c8 c9 c10 c11 c12

%% Altitude Control
e7 = Zd - Z;
e8 = Z_dot - Zd_dot - c7*e7;

U1 = m/cos(Phi)/cos(Tta)*(e7+g+Zd_ddot-c8*e8+c7*(Zd_dot-Z_dot));
%% x Motion Control
e9  = Xd - X;
e10 = X_dot - Xd_dot - c9*e9;

Ux = m/U1*(e9+Xd_ddot-c10*e10+c9*(Xd_dot-X_dot));
Ux_dot = (Ux - Uxy(1))/dt;
Ux_ddot = (Ux - Uxy2(1))/2/dt;
%% y Motion Control
e11 = Yd - Y;
e12 = Y_dot - Yd_dot - c11*e11;

Uy = m/U1*(e11+Yd_ddot-c12*e12+c11*(Yd_dot-Y_dot));
Uy_dot = (Uy - Uxy(2))/dt;
Uy_ddot = (Uy - Uxy2(2))/2/dt;
%% Roll Control
Phid = -Uy;
Phid_dot = -Uy_dot;
Phid_ddot = -Uy_ddot;

e1 = Phid - Phi;
e2 = Phi_dot - Phid_dot - c1*e1;

c1 = 2; c2 = 2;

U2 = 1/b1*(e1-a1*Tta_dot*Psi_dot + Phid_ddot-c2*e2+c1*(Phid_dot-Phi_dot));
%% Pitch Control
Ttad = Ux;
Ttad_dot = Ux_dot;
Ttad_ddot = Ux_ddot;

e3 = Ttad - Tta;
e4 = Tta_dot - Ttad_dot - c3*e3;

c3 = 2; c4 = 2;

U3 = 1/b2*(e3-a2*Phi_dot*Psi_dot+Ttad_ddot-c4*e4+c3*(Ttad_dot-Tta_dot));
%% Yaw Control
e5 = Psid - Psi;
e6 = Psi_dot - Psid_dot - c5*e5;

U4 = 1/b3*(e5-a3*Phi_dot*Tta_dot+Psid_ddot-c6*e6+c5*(Psid_dot-Psi_dot));
%% Control Inputs
U = [U1;U2;U3;U4];
Uxy = [Ux;Uy];

end
