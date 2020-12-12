function [pl] = fnc_momentum(q,dq,Mass,L)

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

pl = zeros(3,1);

  pl(1,1)=dxW*mW + mCM2*(dxW + dtheta1*((L2*cos(theta1 + theta2))/2 + L1*cos(theta1)) + (L2*dtheta2*...
         cos(theta1 + theta2))/2) + mCM1*(dxW + (L1*dtheta1*cos(theta1))/2) + mR*(dxW + dtheta2*(L2*cos(theta1 +...
          theta2) + L3*cos(theta1 + theta2 + theta3)) + dtheta1*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*...
         cos(theta1 + theta2 + theta3)) + L3*dtheta3*cos(theta1 + theta2 + theta3));
  pl(2,1)=0;
  pl(3,1)=- mCM2*(dtheta1*((L2*sin(theta1 + theta2))/2 + L1*sin(theta1)) + (L2*dtheta2*sin(theta1 +...
          theta2))/2) - mR*(dtheta2*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)) + dtheta1*(L2*sin(theta1 +...
          theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) + L3*dtheta3*sin(theta1 + theta2 +...
          theta3)) - (L1*dtheta1*mCM1*sin(theta1))/2;

 