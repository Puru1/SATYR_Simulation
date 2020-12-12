function M_mass = mass(t,y)

L1 = 1;
L2 = 1;
Mw = 1;
Mr  =1;
Mm = 1;
Iw = 1;
Ir = 1;
R = 1;
g = 9.81;
alpha = 0;

%Solving for M
eqs_M(1,1) = (Mm + Mr + Mw)*y(11) + ((Mr*(2*R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + 2*L2*cos(y(2) + y(3)) + 2*L1*cos(y(2) + y(3) + y(4))))/2)*y(12) + ((Mr*(2*R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + 2*L2*cos(y(2) + y(3)) + 2*L1*cos(y(2) + y(3) + y(4))))/2)*y(13) + ((Mr*(2*R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + 2*L1*cos(y(2) + y(3) + y(4))))/2)*y(14) + Mr*R*cos(y(2) - alpha + y(3) + y(4) + y(5))*y(15) - (Mr*y(9)*(2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + 2*L1*sin(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9))))/2 - (Mr*y(7)*(2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + 2*L1*sin(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + 2*L2*sin(y(2) + y(3))*(y(7) + y(8))))/2 - (Mr*y(8)*(2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + 2*L1*sin(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + 2*L2*sin(y(2) + y(3))*(y(7) + y(8))))/2 - Mr*R*y(10)*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10));
eqs_M(2,1) = ((Mr*(2*R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + 2*L2*cos(y(2) + y(3)) + 2*L1*cos(y(2) + y(3) + y(4))))/2)*y(11) + (Ir + Iw + (Mr*(2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))^2 + 2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4)))^2))/2)*y(12) + (Ir + (Mr*(2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))^2 + 2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4)))^2))/2)*y(13) + (Ir + (Mr*(2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L1*cos(y(2) + y(3) + y(4)))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4))) + 2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L1*sin(y(2) + y(3) + y(4)))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))))/2)*y(14) + (Ir + (Mr*(2*R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4))) + 2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))))/2)*y(15) + (Mr*(2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*sin(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*sin(y(2) + y(3))*(y(7) + y(8)))*(y(6) + R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8))) - 2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8)))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*sin(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*sin(y(2) + y(3))*(y(7) + y(8)))))/2 - (Mr*y(10)*(2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(6) + R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8))) - 2*R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*sin(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*sin(y(2) + y(3))*(y(7) + y(8))) + 2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4)))*(y(7) + y(8) + y(9) + y(10)) - 2*R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))*(y(7) + y(8) + y(9) + y(10))))/2 - (Mr*y(7)*(2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))*(y(6) + R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8))) - 2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8)))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))))/2 - (Mr*y(8)*(2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))*(y(6) + R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8))) - 2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8)))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))))/2 - (Mr*y(9)*(2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*sin(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4))) - 2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4))) - 2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L1*cos(y(2) + y(3) + y(4)))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*sin(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*sin(y(2) + y(3))*(y(7) + y(8))) + 2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L1*sin(y(2) + y(3) + y(4)))*(y(6) + R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8)))))/2 - Mr*g*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)));
eqs_M(3,1) = ((Mr*(2*R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + 2*L2*cos(y(2) + y(3)) + 2*L1*cos(y(2) + y(3) + y(4))))/2)*y(11) + (Ir + (Mr*(2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))^2 + 2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4)))^2))/2)*y(12) + (Ir + (Mr*(2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))^2 + 2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4)))^2))/2)*y(13) + (Ir + (Mr*(2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L1*cos(y(2) + y(3) + y(4)))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4))) + 2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L1*sin(y(2) + y(3) + y(4)))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))))/2)*y(14) + (Ir + (Mr*(2*R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4))) + 2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))))/2)*y(15) + (Mr*(2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*sin(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*sin(y(2) + y(3))*(y(7) + y(8)))*(y(6) + R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8))) - 2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8)))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*sin(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*sin(y(2) + y(3))*(y(7) + y(8)))))/2 - (Mr*y(10)*(2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(6) + R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8))) - 2*R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*sin(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*sin(y(2) + y(3))*(y(7) + y(8))) + 2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4)))*(y(7) + y(8) + y(9) + y(10)) - 2*R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))*(y(7) + y(8) + y(9) + y(10))))/2 - (Mr*y(7)*(2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))*(y(6) + R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8))) - 2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8)))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))))/2 - (Mr*y(8)*(2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))*(y(6) + R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8))) - 2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8)))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))))/2 - (Mr*y(9)*(2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*sin(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4))) - 2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4))) - 2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L1*cos(y(2) + y(3) + y(4)))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*sin(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*sin(y(2) + y(3))*(y(7) + y(8))) + 2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L1*sin(y(2) + y(3) + y(4)))*(y(6) + R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8)))))/2 - Mr*g*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)));
eqs_M(4,1) = ((Mr*(2*R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + 2*L1*cos(y(2) + y(3) + y(4))))/2)*y(11) + (Ir + (Mr*(2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L1*cos(y(2) + y(3) + y(4)))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4))) + 2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L1*sin(y(2) + y(3) + y(4)))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))))/2)*y(12) + (Ir + (Mr*(2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L1*cos(y(2) + y(3) + y(4)))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4))) + 2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L1*sin(y(2) + y(3) + y(4)))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))))/2)*y(13) + (Ir + (Mr*(2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L1*cos(y(2) + y(3) + y(4)))^2 + 2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L1*sin(y(2) + y(3) + y(4)))^2))/2)*y(14) + (Ir + (Mr*(2*R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L1*cos(y(2) + y(3) + y(4))) + 2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L1*sin(y(2) + y(3) + y(4)))))/2)*y(15) + (Mr*y(7)*(2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L1*sin(y(2) + y(3) + y(4)))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8))) - 2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L1*sin(y(2) + y(3) + y(4)))*(y(6) + R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8)))))/2 - (Mr*y(10)*(2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(6) + R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8))) - 2*R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*sin(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*sin(y(2) + y(3))*(y(7) + y(8))) + 2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L1*cos(y(2) + y(3) + y(4)))*(y(7) + y(8) + y(9) + y(10)) - 2*R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L1*sin(y(2) + y(3) + y(4)))*(y(7) + y(8) + y(9) + y(10))))/2 - Mr*g*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L1*sin(y(2) + y(3) + y(4))) - (Mr*(2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*sin(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*sin(y(2) + y(3))*(y(7) + y(8))) - 2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*sin(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)))*(y(6) + R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8)))))/2 + (Mr*y(8)*(2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L1*sin(y(2) + y(3) + y(4)))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8))) - 2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L1*sin(y(2) + y(3) + y(4)))*(y(6) + R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8)))))/2 + (Mr*y(9)*(2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L1*cos(y(2) + y(3) + y(4)))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*sin(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*sin(y(2) + y(3))*(y(7) + y(8))) - 2*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L1*sin(y(2) + y(3) + y(4)))*(y(6) + R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8))) + 2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L1*sin(y(2) + y(3) + y(4))) - 2*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L1*cos(y(2) + y(3) + y(4)))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*sin(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)))))/2 ;
eqs_M(5,1) = Mr*R*cos(y(2) - alpha + y(3) + y(4) + y(5))*y(11) + (Ir + (Mr*(2*R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4))) + 2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))))/2)*y(12) + (Ir + (Mr*(2*R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L2*cos(y(2) + y(3)) + L1*cos(y(2) + y(3) + y(4))) + 2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L2*sin(y(2) + y(3)) + L1*sin(y(2) + y(3) + y(4)))))/2)*y(13) + (Ir + (Mr*(2*R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5)) + L1*cos(y(2) + y(3) + y(4))) + 2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5)) + L1*sin(y(2) + y(3) + y(4)))))/2)*y(14) + (Ir + (Mr*(2*R^2*cos(y(2) - alpha + y(3) + y(4) + y(5))^2 + 2*R^2*sin(y(2) - alpha + y(3) + y(4) + y(5))^2))/2)*y(15) - (Mr*(2*R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*sin(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*sin(y(2) + y(3))*(y(7) + y(8)))*(y(7) + y(8) + y(9) + y(10)) - 2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10))*(y(6) + R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8)))))/2 - (Mr*y(7)*(2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(6) + R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8))) - 2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8)))))/2 - (Mr*y(8)*(2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(6) + R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8))) - 2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8)))))/2 - (Mr*y(10)*(2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(6) + R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8))) - 2*R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*sin(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*sin(y(2) + y(3))*(y(7) + y(8)))))/2 - (Mr*y(9)*(2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(6) + R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*cos(y(2) + y(3))*(y(7) + y(8))) - 2*R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*cos(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9))) + 2*R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*sin(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9))) - 2*R*cos(y(2) - alpha + y(3) + y(4) + y(5))*(R*sin(y(2) - alpha + y(3) + y(4) + y(5))*(y(7) + y(8) + y(9) + y(10)) + L1*sin(y(2) + y(3) + y(4))*(y(7) + y(8) + y(9)) + L2*sin(y(2) + y(3))*(y(7) + y(8)))))/2 - Mr*R*g*sin(y(2) - alpha + y(3) + y(4) + y(5));
M_mass = jacobian(eqs_M, y(11:15));
end