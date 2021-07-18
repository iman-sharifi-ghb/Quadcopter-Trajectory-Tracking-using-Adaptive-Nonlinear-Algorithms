function [DesPosition, DesAttitude] = CreateDesiredTrajectory(t)
r = 2; w = 2*pi/40;a = deg2rad(5);
Xd = r*cos(w*t);%1;
Yd = r*sin(w*t);%2;
Zd = t/10;

dXd = -r*w*sin(w*t);
dYd = r*w*cos(w*t);
dZd = 1/10;

ddXd = -r*w^2*cos(w*t);
ddYd = -r*w^2*sin(w*t);
ddZd = 0;

phid = 0;
Ttad = 0;
Psid = a*sin(w*t);

dphid = 0;
dTtad = 0;
dPsid = a*w*cos(w*t);

ddphid = 0;
ddTtad = 0;
ddPsid = -a*w^2*sin(w*t);

DesAttitude = [phid;dphid;ddphid;Ttad;dTtad;ddTtad;Psid;dPsid;ddPsid];

DesPosition = [Zd;dZd;ddZd;Xd;dXd;ddXd;Yd;dYd;ddYd];
end

