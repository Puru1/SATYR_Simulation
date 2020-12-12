function [JvR] = fcn_JvR(q,L)

xW = q(1);
theta1 = q(2);
theta2 = q(3);
theta3 = q(4);
L1 = L(1);
L2 = L(2);
L3 = L(3);

JvR = zeros(3,4);

  JvR(1,1)=1;
  JvR(1,2)=L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3);
  JvR(1,3)=L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3);
  JvR(1,4)=L3*cos(theta1 + theta2 + theta3);
  JvR(2,1)=0;
  JvR(2,2)=0;
  JvR(2,3)=0;
  JvR(2,4)=0;
  JvR(3,1)=0;
  JvR(3,2)=- L2*sin(theta1 + theta2) - L1*sin(theta1) - L3*sin(theta1 + theta2 + theta3);
  JvR(3,3)=- L2*sin(theta1 + theta2) - L3*sin(theta1 + theta2 + theta3);
  JvR(3,4)=-L3*sin(theta1 + theta2 + theta3);

end
 