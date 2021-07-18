function [A,B,C,D]=State_Space()

global m g
global a1 a2 a3
global b1 b2 b3

syms x1 x2 x3 x4 x5 x6
syms x7 x8 x9 x10 x11 x12
syms U1 U2 U3 U4

Ux = cos(x1)*sin(x3)*cos(x5)+sin(x1)*sin(x5);
Uy = cos(x1)*sin(x3)*sin(x5)-sin(x1)*cos(x5);

dx1 = x2;
dx2 = x4*x6*a1+ U2*b1;
dx3 = x4;
dx4 = x2*x6*a2+ U3*b2;
dx5 = x6;
dx6 = x2*x4*a3+ U4*b3;
dx7 = x8;
dx8 = U1/m*(cos(x1)*cos(x3))-g;
dx9 = x10;
dx10 = U1/m*Ux;
dx11 = x12;
dx12 = U1/m*Uy;

x = [x1;x2;x3;x4;x5;x6;x7;x8;x9;x10;x11;x12];
dx = [dx1;dx2;dx3;dx4;dx5;dx6;dx7;dx8;dx9;dx10;dx11;dx12];
u = [U1;U2;U3;U4];

A = jacobian(dx,x);
A = simplify(A);
B = jacobian(dx,u);
B = simplify(B);

n = length(x);
r = length(u);

State_Input_Vars = [x.', u.'];
xeq = [zeros(6,1);x7;0;x9;0;x11;0];
ueq = [m*g;0;0;0];
eq_point = [xeq.', ueq.'];

A = subs(A, State_Input_Vars, eq_point);
B = subs(B, State_Input_Vars, eq_point);

A = vpa(A,6);
B = vpa(B,6);
A = double(A);
B = double(B);
C = zeros(n,1);
D = zeros(r,1);

end

