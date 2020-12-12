function dX = SimpleSegway(t,X)

global g mW mR IW IR L1 L2 R
xW = X(1); 
theta1 = X(2); 
theta2 = X(3);
dxW = X(4); 
dtheta1 = X(5); 
dtheta2 = X(6);

H = [[  mR + mW + IW/R^2,                                                          L1*mR*cos(theta1),                                                          L2*mR*cos(theta2)]
[ L1*mR*cos(theta1),                       (mR*(2*L1^2*cos(theta1)^2 + 2*L1^2*sin(theta1)^2))/2, (mR*(2*L1*L2*cos(theta1)*cos(theta2) + 2*L1*L2*sin(theta1)*sin(theta2)))/2]
[ L2*mR*cos(theta2), (mR*(2*L1*L2*cos(theta1)*cos(theta2) + 2*L1*L2*sin(theta1)*sin(theta2)))/2,                  IR + (mR*(2*L2^2*cos(theta2)^2 + 2*L2^2*sin(theta2)^2))/2]];

C = [  - L1*mR*sin(theta1)*dtheta1^2 - L2*mR*sin(theta2)*dtheta2^2
 -L1*mR*(- L2*sin(theta1 - theta2)*dtheta2^2 + g*sin(theta1))
   -L2*mR*(L1*sin(theta1 - theta2)*dtheta1^2 + g*sin(theta2))];


tau = -0.5*[dxW; dtheta1; dtheta2];
ddq = H\(tau - C);
 
dX = [[dxW; dtheta1; dtheta2]; [ddq(1); ddq(2); ddq(3)]];

end