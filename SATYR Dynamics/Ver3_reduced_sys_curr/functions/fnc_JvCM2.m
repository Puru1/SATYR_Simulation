function [JvCM2] = fcn_JvCM2(q,L)

xW = q(1);
theta1 = q(2);
theta2 = q(3);
theta3 = q(4);
L1 = L(1);
L2 = L(2);
L3 = L(3);

JvCM2 = zeros(3,4);

  JvCM2(1,1)=1;
  JvCM2(1,2)=(L2*cos(theta1 + theta2))/2 + L1*cos(theta1);
  JvCM2(1,3)=(L2*cos(theta1 + theta2))/2;
  JvCM2(1,4)=0;
  JvCM2(2,1)=0;
  JvCM2(2,2)=0;
  JvCM2(2,3)=0;
  JvCM2(2,4)=0;
  JvCM2(3,1)=0;
  JvCM2(3,2)=- (L2*sin(theta1 + theta2))/2 - L1*sin(theta1);
  JvCM2(3,3)=-(L2*sin(theta1 + theta2))/2;
  JvCM2(3,4)=0;

 