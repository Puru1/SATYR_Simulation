function [H] = fnc_H(q,Mass,L1,R)

xW = q(1);
theta1 = q(2);
mW = Mass(1);
mCM = Mass(2);
g = 9.81;
H = zeros(2,2);

  H(1,1)=mCM + mW + 3/(4000*R^2);
  H(1,2)=L1*mCM*cos(theta1);
  H(2,1)=L1*mCM*cos(theta1);
  H(2,2)=L1^2*mCM*cos(theta1)^2 + L1^2*mCM*sin(theta1)^2 + 4933/25000;

 