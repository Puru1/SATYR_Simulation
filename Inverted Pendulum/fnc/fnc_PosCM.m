function [PosCM] = fnc_PosCM(q,L1)

PosCM = zeros(3,1);

  PosCM(1,1)=xW + L1*sin(theta1);
  PosCM(2,1)=0;
  PosCM(3,1)=L1*cos(theta1);

 