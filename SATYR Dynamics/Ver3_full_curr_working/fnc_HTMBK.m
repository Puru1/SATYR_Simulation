function [HTMBK] = fnc_HTMBK(q,L)

HTMBK = zeros(4,4);
xW = q(1);
theta1 = q(2);
theta2 = q(3);
theta3 = q(4);

L1 = L(1);
L2 = L(2);
L3 = L(3);

  HTMBK(1,1)=cos(theta1 + theta2);
  HTMBK(1,2)=0;
  HTMBK(1,3)=-sin(theta1 + theta2);
  HTMBK(1,4)=xW - L1*sin(theta1);
  HTMBK(2,1)=0;
  HTMBK(2,2)=1;
  HTMBK(2,3)=0;
  HTMBK(2,4)=0;
  HTMBK(3,1)=sin(theta1 + theta2);
  HTMBK(3,2)=0;
  HTMBK(3,3)=cos(theta1 + theta2);
  HTMBK(3,4)=L1*cos(theta1);
  HTMBK(4,1)=0;
  HTMBK(4,2)=0;
  HTMBK(4,3)=0;
  HTMBK(4,4)=1;

 