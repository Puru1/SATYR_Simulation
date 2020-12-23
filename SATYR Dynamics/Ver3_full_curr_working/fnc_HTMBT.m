function [HTMBT] = fnc_HTMBT(q,L)

HTMBT = zeros(4,4);
xW = q(1);
theta1 = q(2);
theta2 = q(3);
theta3 = q(4);

L1 = L(1);
L2 = L(2);
L3 = L(3);

  HTMBT(1,1)=cos(theta1 + theta2 + theta3);
  HTMBT(1,2)=0;
  HTMBT(1,3)=-sin(theta1 + theta2 + theta3);
  HTMBT(1,4)=xW - L2*sin(theta1 + theta2) - L1*sin(theta1) - L3*sin(theta1 + theta2 + theta3);
  HTMBT(2,1)=0;
  HTMBT(2,2)=1;
  HTMBT(2,3)=0;
  HTMBT(2,4)=0;
  HTMBT(3,1)=sin(theta1 + theta2 + theta3);
  HTMBT(3,2)=0;
  HTMBT(3,3)=cos(theta1 + theta2 + theta3);
  HTMBT(3,4)=L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3);
  HTMBT(4,1)=0;
  HTMBT(4,2)=0;
  HTMBT(4,3)=0;
  HTMBT(4,4)=1;

 