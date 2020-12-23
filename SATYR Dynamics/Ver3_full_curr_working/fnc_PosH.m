function [PosH] = fnc_PosH(q,L)

PosH = zeros(3,1);

  PosH(1,1)=xW - L2*sin(theta1 + theta2) - L1*sin(theta1);
  PosH(2,1)=0;
  PosH(3,1)=L2*cos(theta1 + theta2) + L1*cos(theta1);

 