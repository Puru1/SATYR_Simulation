function [PosR] = fcn_PosR(q,L)

xW = q(1);
theta1 = q(2);
theta2 = q(3);
theta3 = q(4);
L1 = L(1);
L2 = L(2);
L3 = L(3);

PosR = zeros(3,1);

  PosR(1,1)=xW + L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3);
  PosR(2,1)=0;
  PosR(3,1)=L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3);

 