function [U, Uxy] = SMC_Controller(t, x, y, Uxy)

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
global S S_dot
global m g dt
global a1 a2 a3
global b1 b2 b3
global c1 c2 c3 c4 c5 c6
global k1 k2 k3 k4 k5 k6 k7 k8 k9 k10 k11 k12

%% Altitude Control
e4     = Zd - Z;
e4_dot = Zd_dot - Z_dot;
s4     = e4_dot + c4*e4;

k7 = 5; k8 = 3; c4 = 5;

U1 = m/cos(Phi)/cos(Tta)*(k7*SignApprox(s4, 3, 2)+k8*s4+g+Zd_ddot+c4*e4_dot);
%% x Motion Control
e5     = Xd - X;
disp(['Error x: ' num2str(e5)])
e5_dot = Xd_dot - X_dot;
s5     = e5_dot + c5*e5;
k9 = 5;k10 = 1;c5 = 5;

Ux     = m/U1*(k9*SignApprox(s5, 3, 2)+k10*s5+Xd_ddot+c5*e5_dot);
Ux_dot = (Ux - Uxy(1))/dt;
%% y Motion Control
e6     = Yd - Y;
e6_dot = Yd_dot - Y_dot;
s6     = e6_dot + c6*e6;

k11 = 5; k12 = 1; c6 = 5;
Uy     = m/U1*(k11*SignApprox(s6, 3, 2)+k12*s6+Yd_ddot+c6*e6_dot);
Uy_dot = (Uy - Uxy(2))/dt;
%% Roll Control
e1     = -Uy - Phi;
e1_dot = -Uy_dot - Phi_dot;
s1     = e1_dot + c1*e1;

k1 = 3; k2 = 0; c1 = 10;
U2 = 1/b1*(k1*SignApprox(s1, 3, 2)+k2*s1-a1*Tta_dot*Psi_dot+Phid_ddot+c1*e1_dot);
%% Pitch Control
e2     = Ux - Tta;
e2_dot = Ux_dot - Tta_dot;
s2     = e2_dot + c2*e2;

k3 = 3; k4 = 0; c2 = 10;
U3 = 1/b2*(k3*SignApprox(s2, 3, 2)+k4*s2-a2*Phi_dot*Psi_dot+Ttad_ddot+c2*e2_dot);
%% Yaw Control
e3      = Psid - Psi;
e3_dot  = Psid_dot - Psi_dot;
s3      = e3_dot + c3*e3;

% disp([num2str(Psid) ' and ' num2str(Psi)])


k5 = 0.1;k6 = 5; c3 = 5;

U4 = 1/b3*(k5*SignApprox(s2, 3, 2)+k6*s3-a3*Phi_dot*Tta_dot+Psid_ddot+c3*e3_dot);
%2- PID: K1*e3+K2*e3_dot+K3*e3_ddot;K1 = 0;K2 = 0;K3 = 1;
%3- k51*s3/(abs(s3)+delta)+;delta = 0.3;
%% Control Inputs
U = [U1;U2;U3;U4];
Uxy = [Ux;Uy];

%% Save Slide Surfaces
e3_ddot = Psid_ddot - Psi_ddot;
s3_dot  = e3_ddot + c3*e3_dot;
e4_ddot = Zd_ddot - Z_ddot;
s4_dot  = e4_ddot + c4*e4_dot;
e5_ddot = Xd_ddot - X_ddot;
s5_dot  = e5_ddot + c5*e5_dot;
e6_ddot = Yd_ddot - Y_ddot;
s6_dot  = e6_ddot + c6*e6_dot;
S = [S;[s3,s4,s5,s6]];
S_dot = [S_dot;[s3_dot,s4_dot,s5_dot,s6_dot]];

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

