function [C] = fnc_C(q,p)

C = zeros(2,1);

  C(1,1)=-L1*dtheta1^2*mCM*sin(theta1);
  C(2,1)=-L1*g*mCM*sin(theta1);

 