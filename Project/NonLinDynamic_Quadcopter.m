function [X_dot] = NonLinDynamic_Quadcopter(x,U)

global m g
global a1 a2 a3
global b1 b2 b3

X_dot    = zeros(12,1);

Ux = cos(x(1))*sin(x(3))*cos(x(5))+sin(x(1))*sin(x(5));
Uy = cos(x(1))*sin(x(3))*sin(x(5))-sin(x(1))*cos(x(5));

X_dot(1) = x(2);
X_dot(2) = x(4)*x(6)*a1+ U(2)*b1;
X_dot(3) = x(4);
X_dot(4) = x(2)*x(6)*a2+ U(3)*b2;
X_dot(5) = x(6);
X_dot(6) = x(2)*x(4)*a3+ U(4)*b3;
X_dot(7) = x(8);
X_dot(8) = U(1)/m*(cos(x(1))*cos(x(3)))-g;
X_dot(9) = x(10);
X_dot(10)= U(1)/m*Ux;
X_dot(11)= x(12);
X_dot(12)= U(1)/m*Uy;

end

