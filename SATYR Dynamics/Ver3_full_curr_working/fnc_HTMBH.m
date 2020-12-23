function [HTMBH] = fnc_HTMBH(q,L)

HTMBH = zeros(4,4);
xW = q(1);
theta1 = q(2);
theta2 = q(3);
theta3 = q(4);

L1 = L(1);
L2 = L(2);
L3 = L(3);

  HTMBH(1,1)=cos(theta1 + theta2 + theta3);
  HTMBH(1,2)=0;
  HTMBH(1,3)=-sin(theta1 + theta2 + theta3);
  HTMBH(1,4)=xW - L2*sin(theta1 + theta2) - L1*sin(theta1);
  HTMBH(2,1)=0;
  HTMBH(2,2)=1;
  HTMBH(2,3)=0;
  HTMBH(2,4)=0;
  HTMBH(3,1)=sin(theta1 + theta2 + theta3);
  HTMBH(3,2)=0;
  HTMBH(3,3)=cos(theta1 + theta2 + theta3);
  HTMBH(3,4)=L2*cos(theta1 + theta2) + L1*cos(theta1);
  HTMBH(4,1)=0;
  HTMBH(4,2)=0;
  HTMBH(4,3)=0;
  HTMBH(4,4)=1;

 