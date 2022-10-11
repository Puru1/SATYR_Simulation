function [H] = fnc_H(q,p)

H = zeros(2,2);

  H(1,1)=mCM + mW;
  H(1,2)=L1*mCM*cos(theta1);
  H(2,1)=L1*mCM*cos(theta1);
  H(2,2)=L1^2*mCM*cos(theta1)^2 + L1^2*mCM*sin(theta1)^2;

 