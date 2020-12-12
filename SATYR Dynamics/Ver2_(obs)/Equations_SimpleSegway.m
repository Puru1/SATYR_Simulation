clc;
close all;
clear all;

syms xW theta1 theta2;
syms dxW dtheta1 dtheta2;
syms ddxW ddtheta ddtheta2;
syms g mW mR IW IR L1 L2 R;

%States
q = [xW; theta1; theta2];
dq = [dxW; dtheta1; dtheta2];
ddq = [ddxW; ddtheta; ddtheta2];

%Energy
dx1 = dxW + dtheta1*L1*cos(theta1);
dxR = dx1 + dtheta2*L2*cos(theta2);
dz1 = -dtheta1*L1*sin(theta1);
dzR = dz1 - dtheta2*L2*sin(theta2);


K_wheel = (mW*dxW^2/2 + IW*(dxW/R)^2/2);
K_robot = (mR*(dxR^2 + dzR^2)/2 + IR*dtheta2^2/2); % assuming that theta_r = theta3
Kin = K_wheel + K_robot; 
Pot = mR*g*(L1*cos(theta1) + L2*cos(theta2));

%Lagrangian
L = Kin - Pot;
dLdqi = transpose(jacobian(L,q));
dLddqi = transpose(jacobian(L,dq));
eqs = simplify(jacobian(dLddqi,q)*dq + jacobian(dLddqi,dq)*ddq - dLdqi);

%Equations of motions H(q,dq)*ddq + C = tau
H = jacobian(dLddqi,dq); 
C = simplify(jacobian(dLddqi,q)*dq - dLdqi);











