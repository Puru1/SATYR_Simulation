function [PosR] = fnc_PosCoM_R(q,L)

PosR = zeros(3,1);

  PosR(1,1)=q(1) - (471*cos(q(2) + q(3) + q(4)))/25000 + (7329*sin(q(2) + q(3) + q(4)))/100000 +...
          L(2)*sin(q(2) + q(3)) + L(1)*sin(q(2));
  PosR(2,1)=0;
  PosR(3,1)=(7329*cos(q(2) + q(3) + q(4)))/100000 + (471*sin(q(2) + q(3) + q(4)))/25000 + L(2)*...
         cos(q(2) + q(3)) + L(1)*cos(q(2));

 