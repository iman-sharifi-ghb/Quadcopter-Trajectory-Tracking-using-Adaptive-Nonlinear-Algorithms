% Dynamic Params
global m g
global l b d
global Jx Jy Jz
global a1 a2 a3
global b1 b2 b3

m = 0.65;
g = 9.81;
l = 0.23;
Jx = 7.5e-3;
Jy = 7.5e-3;
Jz = 1.3e-2;
b = 3.13e-5;
d = 7.5e-5;

a1 = (Jy-Jz)/Jx;
a2 = (Jz-Jx)/Jy;
a3 = (Jx-Jy)/Jz;

b1 = l/Jx;
b2 = l/Jy;
b3 = 1/Jz;

% Control Params
global  Kp_phi   Ki_phi   Kd_phi
global  Kp_tta   Ki_tta   Kd_tta 
global  Kp_psi   Ki_psi   Kd_psi
global  Kp_z     Ki_z     Kd_z  
global  Kp_y     Ki_y     Kd_y      
global  Kp_x     Ki_x     Kd_x                                    

Kp_z = 10*m;
Ki_z = 3.5*10^-2;
Kd_z = 10*m;

Kp_psi = 15;
Ki_psi = 0.01;
Kd_psi = 15;

Kp_phi = 1;
Ki_phi = 0.001;
Kd_phi = 0.20;

Kp_y = 10*pi/180;%50*pi/180;
Ki_y = 0.0004;
Kd_y = 6*pi/180;%20*pi/180;

Kp_tta = Kp_phi;
Ki_tta = Ki_phi;
Kd_tta = Kd_phi;

Kp_x = 3*pi/180;
Ki_x = 0.000015;
Kd_x = 5*pi/180;%5*pi/180;


% Reset Integral terms
global    int_e_x  int_e_y int_e_z
global    int_e_phi int_e_tta int_e_psi

int_e_x   = 0;
int_e_y   = 0;
int_e_phi = 0;
int_e_tta = 0;
int_e_psi = 0;
int_e_z   = 0;
