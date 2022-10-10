function [H] = fnc_H(q,L,vMass,Ks)

H = zeros(3,3);

  H(1,1)=vMass(2) + vMass(1) + 3/(4000*L(2)^2);
  H(1,2)=vMass(2)*sin(q(3));
  H(1,3)=q(2)*vMass(2)*cos(q(3));
  H(2,1)=vMass(2)*sin(q(3));
  H(2,2)=vMass(2);
  H(2,3)=0;
  H(3,1)=q(2)*vMass(2)*cos(q(3));
  H(3,2)=0;
  H(3,3)=q(2)^2*vMass(2) + 4933/25000;

 