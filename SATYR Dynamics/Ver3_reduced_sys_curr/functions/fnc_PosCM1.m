function [PosCM1] = fcn_PosCM1(q,L)

xW = q(1);
theta1 = q(2);
theta2 = q(3);
theta3 = q(4);
L1 = L(1);
L2 = L(2);
L3 = L(3);

PosCM1 = zeros(3,1);

  PosCM1(1,1)=xW + (L1*sin(theta1))/2;
  PosCM1(2,1)=0;
  PosCM1(3,1)=(L1*cos(theta1))/2;

 