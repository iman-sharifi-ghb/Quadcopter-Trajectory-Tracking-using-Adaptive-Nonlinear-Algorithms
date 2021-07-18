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
global c7 c8 c9 c10 c11 c12

c = 1;
c1 = c;c2 = c; c3 = 0.2; c4 = c;c5 = c; c6 = c;
c7 = c;c8 = c;c9 = c;c10 = c; c11 = c;c12 = c;