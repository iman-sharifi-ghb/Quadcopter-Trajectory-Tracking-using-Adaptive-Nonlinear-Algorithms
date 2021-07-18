function [DesPosition, DesAttitude] = CreateDesiredTraj(t)

Xd = 1;
Yd = 2;
Zd = 3;

dXd = 0;
dYd = 0;
dZd = 0;

ddXd = 0;
ddYd = 0;
ddZd = 0;

phid = 0;
Ttad = 0;
Psid = 0;%deg2rad(1);

dphid = 0;
dTtad = 0;
dPsid = 0;

ddphid = 0;
ddTtad = 0;
ddPsid = 0;

DesAttitude = [phid;dphid;ddphid;Ttad;dTtad;ddTtad;Psid;dPsid;ddPsid];

DesPosition = [Zd;dZd;ddZd;Xd;dXd;ddXd;Yd;dYd;ddYd];
end

