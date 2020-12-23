function [PosK] = fnc_PosK(q,L)

PosK = zeros(3,1);

  PosK(1,1)=xW - L1*sin(theta1);
  PosK(2,1)=0;
  PosK(3,1)=L1*cos(theta1);

 