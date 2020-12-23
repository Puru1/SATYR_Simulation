function [PosT] = fnc_PosT(q,L)

PosT = zeros(3,1);

  PosT(1,1)=xW - L2*sin(theta1 + theta2) - L1*sin(theta1) - L3*sin(theta1 + theta2 + theta3);
  PosT(2,1)=0;
  PosT(3,1)=L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3);

 