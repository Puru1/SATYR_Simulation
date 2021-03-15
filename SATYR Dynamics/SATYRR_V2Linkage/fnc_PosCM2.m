function [PosCM2] = fnc_PosCM2(q,L)

PosCM2 = zeros(3,1);

  PosCM2(1,1)=q(1) - L(1)*sin(q(2)/2 - q(3)) + (9*L(2)*sin(q(2)/2 + q(3)))/10;
  PosCM2(2,1)=0;
  PosCM2(3,1)=L(1)*cos(q(2)/2 - q(3)) + (9*L(2)*cos(q(2)/2 + q(3)))/10;

 