function [PosK] = fnc_PosK(q,L)

PosK = zeros(3,1);

  PosK(1,1)=q(1) - L(1)*sin(q(2)/2 - q(3));
  PosK(2,1)=0;
  PosK(3,1)=L(1)*cos(q(2)/2 - q(3));

 