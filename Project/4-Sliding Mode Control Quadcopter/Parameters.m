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
global c1 c2 c3 c4 c5 c6
global k1 k2 k3 k4 k5 k6 k7 k8 k9 k10 k11 k12

c = 1;
k = 1;

c1 = c;c2 = c; c3 = 2; c4 = c;c5 = c; c6 = c;

k1 = k;k2 = k;k3 = k;k4 = k;k5 = 1;
k6 = 1;k7 = k;k8 = k;k9 = k;k10 = k;
k11 = k;k12 = k;