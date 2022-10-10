function [PosH] = fnc_PosH(q,L)

PosH = zeros(3,1);

  PosH(1,1)=q(1) + L(2)*sin(q(2) + q(3)) + L(1)*sin(q(2));
  PosH(2,1)=0;
  PosH(3,1)=L(2)*cos(q(2) + q(3)) + L(1)*cos(q(2));

 