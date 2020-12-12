function [PosCM2] = fcn_PosCM2(q,L)

xW = q(1);
theta1 = q(2);
theta2 = q(3);
theta3 = q(4);
L1 = L(1);
L2 = L(2);
L3 = L(3);

PosCM2 = zeros(3,1);

  PosCM2(1,1)=xW + (L2*sin(theta1 + theta2))/2 + L1*sin(theta1);
  PosCM2(2,1)=0;
  PosCM2(3,1)=(L2*cos(theta1 + theta2))/2 + L1*cos(theta1);

 