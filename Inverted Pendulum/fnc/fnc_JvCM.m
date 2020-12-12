function [JvCM] = fnc_JvCM(q,L1)

xW = q(1);
theta1 = q(2);

JvCM = zeros(3,2);

  JvCM(1,1)=1;
  JvCM(1,2)=L1*cos(theta1);
  JvCM(2,1)=0;
  JvCM(2,2)=0;
  JvCM(3,1)=0;
  JvCM(3,2)=-L1*sin(theta1);

 