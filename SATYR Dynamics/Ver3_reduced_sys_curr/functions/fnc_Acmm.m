function [Acmm] = fnc_Acmm(q,dq,Mass,L)

xW = q(1);
theta1 = q(2);
theta2 = q(3);
theta3 = q(4);
dxW = dq(1);
dtheta1 = dq(2);
dtheta2 = dq(3);
dtheta3 = dq(4);
L1 = L(1);
L2 = L(2);
L3 = L(3);
mW = Mass(1);
mCM1 = Mass(2);
mCM2 = Mass(3);
mR = Mass(4);


Acmm = zeros(3,4);

  Acmm(1,1)=mCM1 + mCM2 + mR + mW;
  Acmm(1,2)=mCM2*((L2*cos(theta1 + theta2))/2 + L1*cos(theta1)) + mR*(L2*cos(theta1 + theta2) + L1*...
         cos(theta1) + L3*cos(theta1 + theta2 + theta3)) + (L1*mCM1*cos(theta1))/2;
  Acmm(1,3)=mR*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)) + (L2*mCM2*cos(theta1 + theta2))/2;
  Acmm(1,4)=L3*mR*cos(theta1 + theta2 + theta3);
  Acmm(2,1)=0;
  Acmm(2,2)=0;
  Acmm(2,3)=0;
  Acmm(2,4)=0;
  Acmm(3,1)=0;
  Acmm(3,2)=- mR*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 +...
          theta3)) - mCM2*((L2*sin(theta1 + theta2))/2 + L1*sin(theta1)) - (L1*mCM1*sin(theta1))/2;
  Acmm(3,3)=- mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)) - (L2*mCM2*...
         sin(theta1 + theta2))/2;
  Acmm(3,4)=-L3*mR*sin(theta1 + theta2 + theta3);

 