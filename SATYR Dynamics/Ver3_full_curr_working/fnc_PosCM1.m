function [PosCM1] = fnc_PosCM1(q,L)

PosCM1 = zeros(3,1);
xW = q(1);
theta1 = q(2);
theta2 = q(3);
theta3 = q(4);

L1 = L(1);
L2 = L(2);
L3 = L(3);

  PosCM1(1,1)=xW - (2169*L1*sin(theta1))/10000;
  PosCM1(2,1)=0;
  PosCM1(3,1)=(2169*L1*cos(theta1))/10000;

 