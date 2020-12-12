%% Double-link Pendulum 
clear all;

%Params
syms xw theta1 theta2 xw_d theta1_d theta2_d xw_dd theta1_dd theta2_dd;
syms L1 L2 g mw m1 m2 Iw R;


q = [xw; theta1; theta2];
q_d = [xw_d; theta1_d; theta2_d];
q_dd = [xw_dd; theta1_dd; theta2_dd];

%Relative angles
%x1 = L1*sin(theta1);
%x2 = x1 + L2*sin(theta1 + theta2);

%y1 = -L1*cos(theta1);
%y2 = y1 - L2*cos(theta1 + theta2);

%Absolute angles
x1 = L1*sin(theta1);
y1 = L1*cos(theta1);

x2 = x1 + L2*sin(theta2);
y2 = y1 + L2*cos(theta2);

%Vector notation 
p0 = [xw;0];
p1 = [x1;y1];
p2 = [x2;y2];

%J1 = jacobian(p1,q);
%J2 = jacobian(p2,q);

%Potential Energy 
V = m1*g*y1 + m2*g*y2;

%Kinetic Energy 

%Setup derivative values 
x1_d = L1*cos(theta1)*theta1_d + xw_d;
y1_d = -L1*sin(theta1)*theta1_d;

%x2_d = x1_d + (theta1_d + theta2_d)*cos(theta1 + theta2); %relative angle
%y2_d = y1_d + (theta1_d + theta2_d)*sin(theta1 + theta2); %relative angle

x2_d = x1_d + L2*cos(theta2)*theta2_d; %Absolute angle
y2_d = y1_d - L2*sin(theta2_d); %Absolute angle

v0 = [xw_d; 0];
v1 = [x1_d;y1_d];
v2 = [x2_d;y2_d];

v0_vec = norm(v0);
v1_vec = norm(v1);
v2_vec = norm(v2);

%v2 = sqrt((L1^2)*theta1_d^2 + (L2^2)*theta2_d^2 + 2*L1*L2*theta1_d*theta2_d*cos(theta1 - theta2));
%T = (1/2)*m1*v1^2 + (1/2)*m2*v2^2; %Total kinetic energy 

K_wheel = (1/2)*mw*v0_vec^2  + (1/2)*Iw*(xw_d/R)^2;
K_m1 = (1/2)*m1*v1_vec^2;
K_m2 =  (1/2)*m2*v2_vec^2; %Total kinetic energy keeping vector notation

T_vec = K_wheel + K_m1 + K_m2;

%Lagrangian 
%L = T-V;
L = T_vec - V;

%Equations of motion 
dL_dq = jacobian(L,q); %dL/dq

temp_A = jacobian(L,q_d); %dL/dq_d
temp_B = jacobian(temp_A, q); %(d^2)L/(dq_d * dq)
temp_C = jacobian(temp_A, q_d); % (d^2)L/(dq_d^2)
Q = [0;-2*theta1_d;-2*theta2_d];

ddt_dL_dq_d = temp_B*q_d + temp_C*q_dd;
eqs_temp = ddt_dL_dq_d - transpose(dL_dq); %This is the eqs without any zero attached
eqs = ddt_dL_dq_d - transpose(dL_dq) == Q;

%Finding M and C matrix to place into state-space form
coeffs = collect(eqs_temp, q_dd);

%relative angle
%eqs_M(1,1) = (m1*(2*L1^2*cos(theta1)^2 + 2*L1^2*sin(theta1)^2))/2 + (m2*(2*(sin(theta1 + theta2) - L1*sin(theta1))^2 + 2*(cos(theta1 + theta2) + L1*cos(theta1))^2))/2 *theta1_dd + ((m2*(2*cos(theta1 + theta2)*(cos(theta1 + theta2) + L1*cos(theta1)) + 2*sin(theta1 + theta2)*(sin(theta1 + theta2) - L1*sin(theta1))))/2)*theta2_dd;
%eqs_M(2,1) = ((m2*(2*cos(theta1 + theta2)*(cos(theta1 + theta2) + L1*cos(theta1)) + 2*sin(theta1 + theta2)*(sin(theta1 + theta2) - L1*sin(theta1))))/2)*theta1_dd + ((m2*(2*cos(theta1 + theta2)^2 + 2*sin(theta1 + theta2)^2))/2)*theta2_dd;

%absolute angle
eqs_M(1,1) = theta1_dd*(L1^2*m1 + L1^2*m2) + L1*L2*m2*theta2_dd*cos(theta1 - theta2); %only the q_dd terms are put here
eqs_M(2,1) = L1*L2*m2*cos(theta1 - theta2)*theta1_dd + L2^2*m2*theta2_dd;
%M = jacobian(eqs_M, q_dd);
M = jacobian(coeffs, q_dd); 

%Solving for C
temp_difference = eqs_temp - (M*q_dd); % Original equation without zero equals - q_dd terms
C(1,1) = Q(1) - temp_difference(1,1);
C(2,1) = Q(2) - temp_difference(2,1);
C(3,1) = Q(3) - temp_difference(3,1);


