function [f] = fnc_f(q,p)

f = zeros(2,1);

  f(1,1)=(L(1)*tau1 + L(2)*tau1*cos(q(2)) + L(1)^2*L(2)*dtheta1^2*vMass(2)*sin(q(2)) - L(1)*L(2)*g*...
         vMass(2)*cos(q(2))*sin(q(2)))/(L(1)*L(2)*(vMass(2) + vMass(1) - vMass(2)*cos(q(2))^2));
  f(2,1)=-(L(2)*vMass(2)*tau1 + L(2)*vMass(1)*tau1 + L(1)*vMass(2)*tau1*cos(q(2)) - L(1)*L(2)*g*...
         vMass(2)^2*sin(q(2)) - L(1)*L(2)*g*vMass(2)*vMass(1)*sin(q(2)) + L(1)^2*L(2)*dtheta1^2*vMass(2)^2*cos(q(2))*...
         sin(q(2)))/(L(1)^2*L(2)*vMass(2)*(vMass(2) + vMass(1) - vMass(2)*cos(q(2))^2));

 