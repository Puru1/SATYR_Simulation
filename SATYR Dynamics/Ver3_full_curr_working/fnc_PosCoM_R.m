function [PosR] = fnc_PosCoM_R(q,L)

PosR = zeros(3,1);
xW = q(1);
theta1 = q(2);
theta2 = q(3);
theta3 = q(4);

L1 = L(1);
L2 = L(2);
L3 = L(3);

  PosR(1,1)=xW - (471*cos(theta1 + theta2 + theta3))/25000 - (7329*sin(theta1 + theta2 +...
          theta3))/100000 - L2*sin(theta1 + theta2) - L1*sin(theta1);
  PosR(2,1)=0;
  PosR(3,1)=(7329*cos(theta1 + theta2 + theta3))/100000 - (471*sin(theta1 + theta2 +...
          theta3))/25000 + L2*cos(theta1 + theta2) + L1*cos(theta1);

 