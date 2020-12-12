clc;
close all;
clearvars -except angle_of_linearization theta2_num_rad;

syms xW theta1 theta2 theta3
syms dxW dtheta1 dtheta2 dtheta3
syms ddxW ddtheta ddtheta2 ddtheta3 tau1 tau2 tau3
syms g mW mK mH mR IW IK IH IR L1 L2 L3 R m_leg1 m_leg2 m_motor

%States
q = [xW; theta1; theta2; theta3];
dq = [dxW; dtheta1; dtheta2; dtheta3];
ddq = [ddxW; ddtheta; ddtheta2; ddtheta3];

%Positions
x_joint1 = L1*sin(theta1);
x_joint2 = L1*sin(theta1) + L2*sin(theta1+theta2);
xR = L1*sin(theta1) + L2*sin(theta1+theta2) + L3*sin(theta1+theta2+theta3);

z_joint1 = L1*cos(theta1);
z_joint2 = L1*cos(theta1) + L2*cos(theta1+theta2);
zR = L1*cos(theta1) + L2*cos(theta1+theta2) + L3*cos(theta1+theta2+theta3);

%Energy
dx1 = dxW + dtheta1*L1*cos(theta1);
dx2 = dx1 + (dtheta2+dtheta1)*L2*cos(theta2+theta1);
dxR = dx2 + (dtheta3+dtheta2+dtheta1)*L3*cos(theta3+theta2+theta1);

%dxLeg1 = dxW + (1/2)*dtheta1*L1*cos(theta1);
%dxLeg2 = dx1 + (1/2)*(dtheta2+dtheta1)*L2*cos(theta2+theta1);

dz1 = -dtheta1*L1*sin(theta1);
dz2 = dz1 - (dtheta2+dtheta1)*L2*sin(theta2+theta1);
dzR = dz2 - (dtheta3+dtheta2+dtheta1)*L3*sin(theta3+theta2+theta1);

%dzLeg1 = -(1/2)*dtheta1*L1*sin(theta1);
%dzLeg2 = dz1 - (1/2)*(dtheta2+dtheta1)*L2*sin(theta2+theta1);

K_wheel = (mW*dxW^2/2 + IW*(dxW/R)^2/2);
%K_leg1 = m_leg1*(dxLeg1^2 + dzLeg1^2)/2;
K_middle = mK*(dx1^2 + dz1^2)/2 + IK*(dtheta1^2)/2;
%K_leg2 = m_leg2*(dxLeg2^2 + dzLeg2^2)/2;
K_top = mH*(dx2^2 + dz2^2)/2 + IH*(dtheta2^2)/2;
K_robot = (mR*(dxR^2 + dzR^2)/2 + IR*(dtheta3^2)/2); % note that theta_r = theta3
Kin = K_wheel + K_middle + K_top + K_robot;

Pot_wheel = 0;
%Pot_leg1 = m_leg1*g*(.5 * L1*cos(theta1));
Pot_middle = mK*g*(L1*cos(theta1));
%Pot_leg2 = m_leg2*g*(L1*cos(theta1) + .5*L2*cos(theta2+theta1));
Pot_top = mH*(L1*cos(theta1) + L2*cos(theta2+theta1));
Pot_robot = mR*g*(L1*cos(theta1) + L2*cos(theta2+theta1) + L3*cos(theta3+theta2+theta3));

Pot = Pot_wheel + Pot_middle + Pot_top + Pot_robot;

%Lagrangian
L = Kin - Pot;
dLdqi = transpose(jacobian(L,q));
dLddqi = transpose(jacobian(L,dq));
eqs = simplify(jacobian(dLddqi,q)*dq + jacobian(dLddqi,dq)*ddq - dLdqi);

%Equations of motions H(q,dq)*ddq + C = tau
H = jacobian(dLddqi,dq)
C = simplify(jacobian(dLddqi,q)*dq - dLdqi)
u = [tau1/R; -tau1; -tau2; -tau3];
tau = [tau1;tau2;tau3];
%u = [0;0;0;0];

%%
%Linearization: dX = A*X + B*tau
H_inv = inv(H);
%f = simplify(H\(u - C));
f = H_inv *(u-C); % The simplify() command and the \ seem to be the problem

%Linearized torques
theta3_num = 0;

r_wk = [x_joint1 0 z_joint1];
r_wh = [x_joint2 0 z_joint2];
r_wR = [xR 0 zR];

r_kh = r_wh-r_wk;
r_kR = r_wR - r_wk;

r_hR = r_wR - r_wh;

g_acc = [0 0 -g];
tau1_lin = cross(r_wk, mK*g_acc) + cross(r_wh, mH*g_acc) + cross(r_wR, mR*g_acc);
tau2_lin = cross(r_kh, mH*g_acc) + cross(r_kR, mR*g_acc);
tau3_lin = cross(r_hR, mR*g_acc);

%%
tau1_lin = - vpa(subs(tau1_lin, [theta1 theta2 theta3 g mR mK mH L1 L2 L3],[angle_of_linearization theta2_num_rad theta3_num 9.81 15 .04 .4 .5 .5 .1]))
tau2_lin = - vpa(subs(tau2_lin, [theta1 theta2 theta3 g mR mK mH L1 L2 L3],[angle_of_linearization theta2_num_rad theta3_num 9.81 15 .04 .4 .5 .5 .1]))
tau3_lin = - vpa(subs(tau3_lin, [theta1 theta2 theta3 g mR mK mH L1 L2 L3],[angle_of_linearization theta2_num_rad theta3_num 9.81 15 .04 .4 .5 .5 .1]))

A = jacobian(f,[q; dq]);
A = vpa(subs(A,[xW theta1 theta2 theta3 dxW dtheta1 dtheta2 dtheta3 tau1 tau2 tau3 g R mW mK mH mR L1 L2 L3 IW IK IH IR],[0 angle_of_linearization theta2_num_rad theta3_num 0 0 0 0 tau1_lin(2) tau2_lin(2) tau3_lin(2) 9.81 .5 .5 .04 .4 15 .5 .5 .1 .0013 10^-4 10^-3 .05]))
B = jacobian(f,tau);
B = vpa(subs(B,[xW theta1 theta2 theta3 dxW dtheta1 dtheta2 dtheta3 tau1 tau2 tau3 g R mW mK mH mR L1 L2 L3 IW IK IH IR],[0 angle_of_linearization theta2_num_rad theta3_num 0 0 0 0 tau1_lin(2) tau2_lin(2) tau3_lin(2) 9.81 .5 .5 .04 .4 15 .5 .5 .1 .0013 10^-4 10^-3 .05]))










