function [PosT] = fnc_PosT(q,L)

PosT = zeros(3,1);

  PosT(1,1)=q(1) + L(1)*sin(q(2) + q(3)) + L(3)*sin(q(3)) - L(2)*sin(q(2) - q(3));
  PosT(2,1)=0;
  PosT(3,1)=L(1)*cos(q(2) + q(3)) + L(3)*cos(q(3)) + L(2)*cos(q(2) - q(3));

 