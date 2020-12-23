function [HTMBcm1] = fnc_HTMBcm1(q,L)

HTMBcm1 = zeros(4,4);
xW = q(1);
theta1 = q(2);
theta2 = q(3);
theta3 = q(4);

L1 = L(1);
L2 = L(2);
L3 = L(3);

  HTMBcm1(1,1)=cos(theta1);
  HTMBcm1(1,2)=0;
  HTMBcm1(1,3)=-sin(theta1);
  HTMBcm1(1,4)=xW - (2169*L1*sin(theta1))/10000;
  HTMBcm1(2,1)=0;
  HTMBcm1(2,2)=1;
  HTMBcm1(2,3)=0;
  HTMBcm1(2,4)=0;
  HTMBcm1(3,1)=sin(theta1);
  HTMBcm1(3,2)=0;
  HTMBcm1(3,3)=cos(theta1);
  HTMBcm1(3,4)=(2169*L1*cos(theta1))/10000;
  HTMBcm1(4,1)=0;
  HTMBcm1(4,2)=0;
  HTMBcm1(4,3)=0;
  HTMBcm1(4,4)=1;

 