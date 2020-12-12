syms theta1(t) theta2(t) theta3(t) 
syms L1 L2 L3

xR_relative = L1*sin(theta1) + L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3);
zR = L1*cos(theta1) + L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3);

theta_t = tan(xR_relative/zR);
theta_t_d = diff(theta_t, t)

