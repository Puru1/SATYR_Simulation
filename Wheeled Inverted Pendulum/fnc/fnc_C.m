function [C] = fnc_C(q,dq,Mass,L1)

xW = q(1);
theta1 = q(2);
dxW = dq(1);
dtheta1 = dq(2);
mW = Mass(1);
mCM = Mass(2);
g = 9.81;

C = zeros(2,1);

  C(1,1)=-L1*dtheta1^2*mCM*sin(theta1);
  C(2,1)=-L1*g*mCM*sin(theta1);

 