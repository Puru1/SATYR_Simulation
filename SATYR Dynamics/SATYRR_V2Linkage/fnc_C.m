function [C] = fnc_C(q,dq,L,vMass,g,Ks)

C = zeros(3,1);

  C(1,1)=dq(3)*vMass(2)*(2*dq(2)*cos(q(3)) - dq(3)*q(2)*sin(q(3)));
  C(2,1)=Ks*q(2) - Ks*L(1) + g*vMass(2)*cos(q(3)) - dq(3)^2*q(2)*vMass(2);
  C(3,1)=q(2)*vMass(2)*(2*dq(2)*dq(3) - g*sin(q(3)));

 