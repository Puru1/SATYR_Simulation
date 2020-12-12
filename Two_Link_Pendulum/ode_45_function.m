function x_dot = ode_45_function(t,y)

%Parameter setup
g = 9.81;
L1 = 2;
L2 = 2;
m1 = 1000;
m2 = 1;

%Solving for M numerically 
M(1,1) = L1^2*m1 + L1^2*m2;
M(1,2) = L1*L2*m2*cos(y(1) - y(2));
M(2,1) = L1*L2*m2*cos(y(1) - y(2));
M(2,2) = L2^2*m2;


%Solving for C numerically 
C(1,1) = -2*y(3) - L1*L2*m2*sin(y(1) - y(2))*y(4)^2 - L1*g*m1*sin(y(1)) - L1*g*m2*sin(y(1));
C(2,1) = -2*y(4) + L1*L2*m2*sin(y(1) - y(2))*y(3)^2 - L2*g*m2*sin(y(2));

x_dd = M\C;

%Output
x_dot_1 = y(3); %y(3) = theta1_d
x_dot_2 = y(4); %y(4) = theta2_d
x_dot_3 = x_dd(1);
x_dot_4 = x_dd(2);

x_dot = [x_dot_1;x_dot_2;x_dot_3;x_dot_4];

end