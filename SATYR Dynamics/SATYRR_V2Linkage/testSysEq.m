syms theta1 theta2 alpha beta L1 L2 real

% L1 = .15;
% L2 = .15;
% alpha = .3;
% beta = 0;

E1 = L1*cos(theta1) + L2*cos(theta2) - alpha == 0;
E2 = L1*sin(theta1) + L2*sin(theta2) - beta == 0;

sol = solve([E1,E2],[theta1,theta2])
sol(1)