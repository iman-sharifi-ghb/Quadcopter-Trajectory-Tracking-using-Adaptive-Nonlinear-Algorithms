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
global K1 K2 %K3 K4 K5 K6 K7 K8 K9 K10 K11 K12
global KI

% K = 1;
% K1 = K;K2 = K;K3 = K;K4 = K;K5 = 1;
% K6 = 1;K7 = K;K8 = K;K9 = K;K10 = K;
% K11 = K;K12 = K;
K1 = 2;K2 = 3;
KI = 6;

global int_e_Z   int_e_X   int_e_Y
global int_e_Phi int_e_Tta int_e_Psi

int_e_Z = 0;   int_e_X = 0;   int_e_Y = 0;
int_e_Phi = 0; int_e_Tta = 0; int_e_Psi = 0;