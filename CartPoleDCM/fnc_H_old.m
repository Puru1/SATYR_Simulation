function [H] = fnc_H(q,p)

H = zeros(2,2);

mCM = p.mCM;
mW = p.mW;
g = p.g;
L1 = p.L1;
xW = q(1);
theta1 = q(2);
dxW = q(3);
dtheta1 = q(4);

  H(1,1)=mCM + mW;
  H(1,2)=L1*mCM*cos(theta1);
  H(2,1)=L1*mCM*cos(theta1);
  H(2,2)=L1^2*mCM*cos(theta1)^2 + L1^2*mCM*sin(theta1)^2 + 4933/25000;

 