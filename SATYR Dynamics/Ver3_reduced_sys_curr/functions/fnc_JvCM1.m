function [JvCM1] = fcn_JvCM1(q,L)

xW = q(1);
theta1 = q(2);
theta2 = q(3);
theta3 = q(4);
L1 = L(1);
L2 = L(2);
L3 = L(3);

JvCM1 = zeros(3,4);

  JvCM1(1,1)=1;
  JvCM1(1,2)=(L1*cos(theta1))/2;
  JvCM1(1,3)=0;
  JvCM1(1,4)=0;
  JvCM1(2,1)=0;
  JvCM1(2,2)=0;
  JvCM1(2,3)=0;
  JvCM1(2,4)=0;
  JvCM1(3,1)=0;
  JvCM1(3,2)=-(L1*sin(theta1))/2;
  JvCM1(3,3)=0;
  JvCM1(3,4)=0;

 