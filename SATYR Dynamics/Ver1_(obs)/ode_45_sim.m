function x_dot = ode_45_sim(t,y)
% parameter setup
%syms L1 L2 Mw Mr Mm Iw Ir R g alpha     real
L1 = 1;
L2 = 1;
Mw = 1.25;
Mr  =1.5;
Mm = 1;
Iw = 2;
Ir = 3;
R = .2;
g = 9.81;
alpha = 0;
Tau1 = 0;
Tau2 = 0;
Tau3 = 0;

%Solving for M numerically
M(1,1) = Mm + Mw + Mr;
M(1,2) = (Mr*(2*R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + 2*L2*cos(y(2) + y(3)) + 2*L1*cos(y(2) + y(3) + y(4))))/2;
M(1,3) = (Mr*(2*R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + 2*L2*cos(y(2) + y(3)) + 2*L1*cos(y(2) + y(3) + y(4))))/2;
M(1,4) = (Mr*(2*R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + 2*L1*cos(y(2) + y(3) + y(4))))/2;
M(1,5) = Mr*R*cos(y(2) - alpha + y(3) + y(4) + y(5));f

M(2,1) = (Mr*(2*R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + 2*L2*cos(y(2) + y(3)) + 2*L1*cos(y(2) + y(3) + y(4))))/2;
M(2,2) = Ir + Iw + (Mr*(2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))^2 + 2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4)))^2))/2;
M(2,3) = Ir + (Mr*(2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))^2 + 2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4)))^2))/2;
M(2,4) = Ir + (Mr*((2*R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + 2*L1*cos(y(2) + y(3) + y(4)))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4))) + (2*R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + 2*L1*sin(y(2) + y(3) + y(4)))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))))/2;
M(2,5) = Ir + (Mr*(2*R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4))) + 2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))))/2;

M(3,1) = (Mr*(2*R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + 2*L2*cos(y(2) + y(3)) + 2*L1*cos(y(2) + y(3) + y(4))))/2;
M(3,2) = Ir + (Mr*(2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))^2 + 2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4)))^2))/2;
M(3,3) = Ir + (Mr*(2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))^2 + 2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4)))^2))/2;
M(3,4) = Ir + (Mr*((2*R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + 2*L1*cos(y(2) + y(3) + y(4)))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4))) + (2*R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + 2*L1*sin(y(2) + y(3) + y(4)))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))))/2;
M(3,5) = Ir + (Mr*(2*R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4))) + 2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))))/2;

M(4,1) = (Mr*(2*R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + 2*L1*cos(y(2) + y(3) + y(4))))/2;
M(4,2) = Ir + (Mr*((2*R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + 2*L1*cos(y(2) + y(3) + y(4)))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4))) + (2*R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + 2*L1*sin(y(2) + y(3) + y(4)))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))))/2;
M(4,3) = Ir + (Mr*((2*R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + 2*L1*cos(y(2) + y(3) + y(4)))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4))) + (2*R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + 2*L1*sin(y(2) + y(3) + y(4)))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))))/2;
M(4,4) = Ir + (Mr*(2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L1*cos(y(2) + y(3) + y(4)))^2 + 2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L1*sin(y(2) + y(3) + y(4)))^2))/2;
M(4,5) = Ir + (Mr*(2*R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L1*cos(y(2) + y(3) + y(4))) + 2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L1*sin(y(2) + y(3) + y(4)))))/2;

M(5,1) = Mr*R*cos(y(2) - alpha + y(3) + y(4) + y(5));
M(5,2) = Ir + (Mr*(2*R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4))) + 2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))))/2;
M(5,3) = Ir + (Mr*(2*R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4))) + 2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))))/2;
M(5,4) = Ir + (Mr*(2*R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L1*cos(y(2) + y(3) + y(4))) + 2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L1*sin(y(2) + y(3) + y(4)))))/2;
M(5,5) = Ir + (Mr*(2*R^2*cos(y(2) - alpha + y(3) + y(4) + y(5))^2 + 2*R^2*sin(y(2) - alpha + y(3) + y(4) + y(5))^2))/2;

%Solving for H numerically
H(1,1) = Tau1/R + (Mr*y(9)*(2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + 2*L1*sin(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9))))/2 + (Mr*y(7)*(2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + 2*L1*sin(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + 2*L2*sin(y(2) + y(3))*(y(7) + y(8))))/2 + (Mr*y(8)*(2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + 2*L1*sin(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + 2*L2*sin(y(2) + y(3))*(y(7) + y(8))))/2 + Mr*R*y(10)*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10));
H(2,1) = L2*Mr*g*sin(y(2) + y(3)) + L1*Mr*g*sin(y(2) + y(3) + y(4)) + Mr*R*g*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L1*L2*Mr*y(9)^2*sin(y(4)) - L1*Mr*R*y(10)^2*sin(alpha - y(5)) + L2*Mr*R*y(9)^2*sin(y(4) - alpha + y(5)) + L2*Mr*R*y(10)^2*sin(y(4) - alpha + y(5)) + 2*L1*L2*Mr*y(7)*y(9)*sin(y(4)) + 2*L1*L2*Mr*y(8)*y(9)*sin(y(4)) - 2*L1*Mr*R*y(7)*y(10)*sin(alpha - y(5)) - 2*L1*Mr*R*y(8)*y(10)*sin(alpha - y(5)) - 2*L1*Mr*R*y(9)*y(10)*sin(alpha - y(5)) + 2*L2*Mr*R*y(7)*y(9)*sin(y(4) - alpha + y(5)) + 2*L2*Mr*R*y(7)*y(10)*sin(y(4) - alpha + y(5)) + 2*L2*Mr*R*y(8)*y(9)*sin(y(4) - alpha + y(5)) + 2*L2*Mr*R*y(8)*y(10)*sin(y(4) - alpha + y(5)) + 2*L2*Mr*R*y(9)*y(10)*sin(y(4) - alpha + y(5));
H(3,1) = Tau1 + L2*Mr*g*sin(y(2) + y(3)) + L1*Mr*g*sin(y(2) + y(3) + y(4)) + Mr*R*g*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L1*L2*Mr*y(9)^2*sin(y(4)) - L1*Mr*R*y(10)^2*sin(alpha - y(5)) + L2*Mr*R*y(9)^2*sin(y(4) - alpha + y(5)) + L2*Mr*R*y(10)^2*sin(y(4) - alpha + y(5)) + 2*L1*L2*Mr*y(7)*y(9)*sin(y(4)) + 2*L1*L2*Mr*y(8)*y(9)*sin(y(4)) - 2*L1*Mr*R*y(7)*y(10)*sin(alpha - y(5)) - 2*L1*Mr*R*y(8)*y(10)*sin(alpha - y(5)) - 2*L1*Mr*R*y(9)*y(10)*sin(alpha - y(5)) + 2*L2*Mr*R*y(7)*y(9)*sin(y(4) - alpha + y(5)) + 2*L2*Mr*R*y(7)*y(10)*sin(y(4) - alpha + y(5)) + 2*L2*Mr*R*y(8)*y(9)*sin(y(4) - alpha + y(5)) + 2*L2*Mr*R*y(8)*y(10)*sin(y(4) - alpha + y(5)) + 2*L2*Mr*R*y(9)*y(10)*sin(y(4) - alpha + y(5));
H(4,1) = Tau2 + L1*Mr*g*sin(y(2) + y(3) + y(4)) + Mr*R*g*sin(y(2) - alpha + y(3) + y(4) + y(5)) - L1*L2*Mr*y(8)^2*sin(y(4)) - L1*Mr*R*y(10)^2*sin(alpha - y(5)) - L2*Mr*R*y(7)^2*sin(y(4) - alpha + y(5)) - L2*Mr*R*y(8)^2*sin(y(4) - alpha + y(5)) - L1*L2*Mr*y(7)^2*sin(y(4)) - 2*L1*L2*Mr*y(7)*y(8)*sin(y(4)) - 2*L1*Mr*R*y(7)*y(10)*sin(alpha - y(5)) - 2*L1*Mr*R*y(8)*y(10)*sin(alpha - y(5)) - 2*L1*Mr*R*y(9)*y(10)*sin(alpha - y(5)) - 2*L2*Mr*R*y(7)*y(8)*sin(y(4) - alpha + y(5));
H(5,1) = Tau3 + Mr*R*(g*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L1*y(7)^2*sin(alpha - y(5)) + L1*y(8)^2*sin(alpha - y(5)) + L1*y(9)^2*sin(alpha - y(5)) - L2*y(7)^2*sin(y(4) - alpha + y(5)) - L2*y(8)^2*sin(y(4) - alpha + y(5)) - 2*L2*y(7)*y(8)*sin(y(4) - alpha + y(5)) + 2*L1*y(7)*y(8)*sin(alpha - y(5)) + 2*L1*y(7)*y(9)*sin(alpha - y(5)) + 2*L1*y(8)*y(9)*sin(alpha - y(5)));

x_dd = M\H;

%Output
x_dot_1 = y(6); %y(3) = y(6)
x_dot_2 = y(7); %y(4) = y(7)
x_dot_3 = y(8); %y(5) = y(8)
x_dot_4 = y(9); %y(5) = y(8)
x_dot_5 = y(10); %y(5) = y(8)
x_dot_6 = x_dd(1,1);
x_dot_7 = x_dd(2,1);
x_dot_8 = x_dd(3,1);
x_dot_9 = x_dd(4,1);
x_dot_10 = x_dd(5,1);


x_dot = [x_dot_1;x_dot_2;x_dot_3;x_dot_4;x_dot_5;x_dot_6;x_dot_7;x_dot_8;x_dot_9;x_dot_10];
end
