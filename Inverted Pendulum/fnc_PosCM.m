function [PosCM] = fnc_PosCM(q,L)

PosCM = zeros(3,1);

  PosCM(1,1)=q(1) + L(1)*sin(q(2));
  PosCM(2,1)=0;
  PosCM(3,1)=L(1)*cos(q(2));

 