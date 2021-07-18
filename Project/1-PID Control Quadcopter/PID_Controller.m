function [U, Uxy] = PID_Controller(X,X_ddot,Uxy)
global  g dt
global  Kp_phi   Ki_phi   Kd_phi
global  Kp_tta   Ki_tta   Kd_tta 
global  Kp_psi   Ki_psi   Kd_psi
global  Kp_z     Ki_z     Kd_z  
global  Kp_y     Ki_y     Kd_y      
global  Kp_x     Ki_x     Kd_x  
global  int_e_phi int_e_tta int_e_psi 
global  int_e_z   int_e_x   int_e_y

% calculate desired state as function of t
x = X(9) ;x_dot = X(10);x_ddot = X_ddot(5);
y = X(11);y_dot = X(12);y_ddot = X_ddot(6);
z = X(7) ;z_dot = X(8);
phi = X(1);phi_dot = X(2);
tta = X(3);tta_dot = X(4);
psi = X(5);psi_dot = X(6);

%% Desired States
%% Desired States
[DesPos, DesAtt] = CreateDesiredTraj(0);

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

%% Calculate XYZ errors
e_x     = Xd - x;
e_dot_x = Xd_dot- x_dot;

e_y     = Yd- y;
e_dot_y = Yd_dot- y_dot;

e_z     = Zd - z;
e_dot_z = Zd_dot - z_dot;

Ux = Kp_x*e_x + Ki_x*int_e_x + Kd_x*e_dot_x;
Ux_dot = (Ux - Uxy(1))/dt;

Uy = -(Kp_y*e_y + Ki_y*int_e_y + Kd_y*e_dot_y);
Uy_dot = (Uy - Uxy(2))/dt;

%% Calculate Angle Errors from Desired Angles 
e_phi     = Uy     - phi;
e_dot_phi = Uy_dot - phi_dot;

e_tta     = Ux     - tta;
e_dot_tta = Ux_dot - tta_dot;

e_psi     = Psid - psi;
e_dot_psi = Psid_dot - psi_dot;

int_e_phi = int_e_phi + e_phi;
int_e_tta = int_e_tta + e_tta;
int_e_psi = int_e_psi + e_psi;
int_e_z   = int_e_z   + e_z;
int_e_x   = int_e_x   + e_x;
int_e_y   = int_e_y   + e_y;

%% PD Loop to Calculate Torques, Thrusts
T = RotI2B(phi, tta, psi)*[0;0;g] + e_z*Kp_z + e_dot_z*Kd_z + int_e_z*Ki_z;
T = max([T(3) 0]);

tau_phi = (e_phi*Kp_phi + e_dot_phi*Kd_phi + int_e_phi*Ki_phi);
tau_tta = (e_tta*Kp_tta + e_dot_tta*Kd_tta + int_e_tta*Ki_tta);
tau_psi = (e_psi*Kp_psi + e_dot_psi*Kd_psi + int_e_psi*Ki_psi);

% disp(['e_phi = ' num2str(e_phi) ', e_tta = ' num2str(e_tta)])
disp(['e_x = ' num2str(e_x) ', e_y = ' num2str(e_y)])
U       = [T tau_phi tau_tta tau_psi]';
Uxy = [Ux;Uy];    
end