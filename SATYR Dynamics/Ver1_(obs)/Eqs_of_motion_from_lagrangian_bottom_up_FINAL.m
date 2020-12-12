%%
clear all;

%finding the equations of motion for side view of Hermees' Chatiot with
%both legs, body tilt, and skates on. 
format compact 
clear;clc;

%constants
syms L1 L2 Mw Mr Mm Iw Ir R g alpha     real
%torques - non conservative forces
syms Tau1 Tau2 Tau3
%state variables
syms Xw    phi_w    theta_1    theta_2    theta_3     real
syms Xw_d  phi_w_d  theta_1_d  theta_2_d  theta_3_d   real
syms Xw_dd phi_w_dd theta_1_dd theta_2_dd theta_3_dd  real

%Xw_d = diff(Xw,1); phi_w_d = diff(phi_w,1); theta_1_d = diff(theta_1,1); theta_2_d = diff(theta_2,1); theta_3_d = diff(theta_3,1); 
%Xw_dd = diff(Xw,2); phi_w_dd = diff(phi_w,2); theta_1_dd = diff(theta_1,2); theta_2_dd = diff(theta_2,2); theta_3_dd = diff(theta_3,2); 

q    = [Xw    ;phi_w    ;theta_1    ;theta_2    ;theta_3   ];
q_d  = [Xw_d  ;phi_w_d  ;theta_1_d  ;theta_2_d  ;theta_3_d ];
q_dd = [Xw_dd ;phi_w_dd ;theta_1_dd ;theta_2_dd ;theta_3_dd];

%angle equations
phi_1   = phi_w   +  theta_1;
phi_1_d = phi_w_d +  theta_1_d;

phi_r   = -(phi_1   + theta_2   + theta_3  );
phi_r_d = -(phi_1_d + theta_2_d + theta_3_d);

%posiiotn of the center of Mass of Robot
Xr = Xw + L2*sin(phi_1) + L1*sin(phi_1 + theta_2) + R*sin(phi_1 + theta_2 + theta_3 - alpha);
Zr =  0 + L2*cos(phi_1) + L1*cos(phi_1 + theta_2) + R*cos(phi_1 + theta_2 + theta_3 - alpha);
%velocity of the center of Mass of Robot
Xr_d = Xw_d + L2*cos(phi_1)*(phi_1_d) + L1*cos(phi_1 + theta_2)*(phi_1_d + theta_2_d) + R*cos(phi_1 + theta_2 + theta_3 - alpha)*(phi_1_d + theta_2_d + theta_3_d);
Zr_d = 0    - L2*sin(phi_1)*(phi_1_d) - L1*sin(phi_1 + theta_2)*(phi_1_d + theta_2_d) - R*sin(phi_1 + theta_2 + theta_3 - alpha)*(phi_1_d + theta_2_d + theta_3_d);
Pr_d = [Xr_d; Zr_d];

%Potential Energy
V = Mr*g*Zr;

%Kinetic energy of the wheel
Kw = (Mw+Mm)/2*(Xw_d)^2 + Iw/2*(phi_w_d)^2;
%Kinetic energy of the robot
Kr = Mr/2*(Xr_d^2 + Zr_d^2) + Ir/2*(phi_r_d)^2;

%Total Kinetic Energy
T = Kw + Kr;

%Lagrangian
L = T-V;

%Equations of motion support
% dL_dq = [t;t;t;t;t];%declairing it as a sym array. It changes later
% for i = 1:length(q)
%     dL_dq(i) = diff(L,q(i));
% end
dL_dq = jacobian(L,q);

%A = dL_dq_d 
% A = [t;t;t;t;t];%declairing it as a sym array. It changes later
% for i = 1:length(q)
%     A(i) = diff(L,q_d(i));
% end
temp_A = jacobian(L,q_d);

%B = diff(A,q) %C = diff(A,q_d)
% B = [t;t;t;t;t]; C = [t;t;t;t;t];%declairing it as a sym array. It changes later
% for i = 1:length(q)
%     B(i) = diff(A(i),q(i));
%     C(i) = diff(A(i),q_d(i));
% end
temp_B = jacobian(temp_A, q);
temp_C = jacobian(temp_A, q_d);

% ddt_dL_dq_d = [t;t;t;t;t];
% for i = 1:length(q)
%     ddt_dL_dq_d(i) = B(i)*q_d(i) + C(i)*q_dd(i);
% end
ddt_dL_dq_d = temp_B*q_d + temp_C*q_dd;

%external non-concervitive forces vector
Q = [Tau1/R; 0; Tau1; Tau2; Tau3];

%Equations of motion 
eqs_temp = ddt_dL_dq_d - transpose(dL_dq); %This is the eqs without any zero attached
eqs = ddt_dL_dq_d - transpose(dL_dq) == Q;

%%
%Finding M and C matrix to place into state-space form
%eqs_simple = simplify(eqs);
eqs_M = collect(eqs_temp, q_dd);
%eqs_M(1,1) = (Mm + Mr + Mw)*Xw_dd + ((Mr*(2*R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + 2*L2*cos(phi_w + theta_1) + 2*L1*cos(phi_w + theta_1 + theta_2)))/2)*phi_w_dd + ((Mr*(2*R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + 2*L2*cos(phi_w + theta_1) + 2*L1*cos(phi_w + theta_1 + theta_2)))/2)*theta_1_dd + ((Mr*(2*R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + 2*L1*cos(phi_w + theta_1 + theta_2)))/2)*theta_2_dd + Mr*R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*theta_3_dd - (Mr*theta_2_d*(2*R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + 2*L1*sin(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d)))/2 - (Mr*phi_w_d*(2*R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + 2*L1*sin(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + 2*L2*sin(phi_w + theta_1)*(phi_w_d + theta_1_d)))/2 - (Mr*theta_1_d*(2*R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + 2*L1*sin(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + 2*L2*sin(phi_w + theta_1)*(phi_w_d + theta_1_d)))/2 - Mr*R*theta_3_d*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d);
%eqs_M(2,1) = ((Mr*(2*R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + 2*L2*cos(phi_w + theta_1) + 2*L1*cos(phi_w + theta_1 + theta_2)))/2)*Xw_dd + (Ir + Iw + (Mr*(2*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*sin(phi_w + theta_1) + L1*sin(phi_w + theta_1 + theta_2))^2 + 2*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*cos(phi_w + theta_1) + L1*cos(phi_w + theta_1 + theta_2))^2))/2)*phi_w_dd + (Ir + (Mr*(2*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*sin(phi_w + theta_1) + L1*sin(phi_w + theta_1 + theta_2))^2 + 2*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*cos(phi_w + theta_1) + L1*cos(phi_w + theta_1 + theta_2))^2))/2)*theta_1_dd + (Ir + (Mr*(2*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*cos(phi_w + theta_1 + theta_2))*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*cos(phi_w + theta_1) + L1*cos(phi_w + theta_1 + theta_2)) + 2*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*sin(phi_w + theta_1 + theta_2))*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*sin(phi_w + theta_1) + L1*sin(phi_w + theta_1 + theta_2))))/2)*theta_2_dd + (Ir + (Mr*(2*R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*cos(phi_w + theta_1) + L1*cos(phi_w + theta_1 + theta_2)) + 2*R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*sin(phi_w + theta_1) + L1*sin(phi_w + theta_1 + theta_2))))/2)*theta_3_dd; 
%eqs_M(3,1) = ((Mr*(2*R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + 2*L2*cos(phi_w + theta_1) + 2*L1*cos(phi_w + theta_1 + theta_2)))/2)*Xw_dd + (Ir + (Mr*(2*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*sin(phi_w + theta_1) + L1*sin(phi_w + theta_1 + theta_2))^2 + 2*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*cos(phi_w + theta_1) + L1*cos(phi_w + theta_1 + theta_2))^2))/2)*phi_w_dd + (Ir + (Mr*(2*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*sin(phi_w + theta_1) + L1*sin(phi_w + theta_1 + theta_2))^2 + 2*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*cos(phi_w + theta_1) + L1*cos(phi_w + theta_1 + theta_2))^2))/2)*theta_1_dd + (Ir + (Mr*(2*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*cos(phi_w + theta_1 + theta_2))*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*cos(phi_w + theta_1) + L1*cos(phi_w + theta_1 + theta_2)) + 2*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*sin(phi_w + theta_1 + theta_2))*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*sin(phi_w + theta_1) + L1*sin(phi_w + theta_1 + theta_2))))/2)*theta_2_dd + (Ir + (Mr*(2*R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*cos(phi_w + theta_1) + L1*cos(phi_w + theta_1 + theta_2)) + 2*R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*sin(phi_w + theta_1) + L1*sin(phi_w + theta_1 + theta_2))))/2)*theta_3_dd + (Mr*(2*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*sin(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*sin(phi_w + theta_1)*(phi_w_d + theta_1_d))*(Xw_d + R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*cos(phi_w + theta_1)*(phi_w_d + theta_1_d)) - 2*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*cos(phi_w + theta_1)*(phi_w_d + theta_1_d))*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*sin(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*sin(phi_w + theta_1)*(phi_w_d + theta_1_d))))/2 - (Mr*theta_3_d*(2*R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(Xw_d + R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*cos(phi_w + theta_1)*(phi_w_d + theta_1_d)) - 2*R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*sin(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*sin(phi_w + theta_1)*(phi_w_d + theta_1_d)) + 2*R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*cos(phi_w + theta_1) + L1*cos(phi_w + theta_1 + theta_2))*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) - 2*R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*sin(phi_w + theta_1) + L1*sin(phi_w + theta_1 + theta_2))*(phi_w_d + theta_1_d + theta_2_d + theta_3_d)))/2 - (Mr*phi_w_d*(2*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*sin(phi_w + theta_1) + L1*sin(phi_w + theta_1 + theta_2))*(Xw_d + R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*cos(phi_w + theta_1)*(phi_w_d + theta_1_d)) - 2*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*cos(phi_w + theta_1)*(phi_w_d + theta_1_d))*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*sin(phi_w + theta_1) + L1*sin(phi_w + theta_1 + theta_2))))/2 - (Mr*theta_1_d*(2*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*sin(phi_w + theta_1) + L1*sin(phi_w + theta_1 + theta_2))*(Xw_d + R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*cos(phi_w + theta_1)*(phi_w_d + theta_1_d)) - 2*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*cos(phi_w + theta_1)*(phi_w_d + theta_1_d))*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*sin(phi_w + theta_1) + L1*sin(phi_w + theta_1 + theta_2))))/2 - (Mr*theta_2_d*(2*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*sin(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d))*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*cos(phi_w + theta_1) + L1*cos(phi_w + theta_1 + theta_2)) - 2*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d))*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*sin(phi_w + theta_1) + L1*sin(phi_w + theta_1 + theta_2)) - 2*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*cos(phi_w + theta_1 + theta_2))*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*sin(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*sin(phi_w + theta_1)*(phi_w_d + theta_1_d)) + 2*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*sin(phi_w + theta_1 + theta_2))*(Xw_d + R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*cos(phi_w + theta_1)*(phi_w_d + theta_1_d))))/2 - Mr*g*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*sin(phi_w + theta_1) + L1*sin(phi_w + theta_1 + theta_2));
%eqs_M(4,1) = ((Mr*(2*R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + 2*L1*cos(phi_w + theta_1 + theta_2)))/2)*Xw_dd + (Ir + (Mr*(2*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*cos(phi_w + theta_1 + theta_2))*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*cos(phi_w + theta_1) + L1*cos(phi_w + theta_1 + theta_2)) + 2*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*sin(phi_w + theta_1 + theta_2))*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*sin(phi_w + theta_1) + L1*sin(phi_w + theta_1 + theta_2))))/2)*phi_w_dd + (Ir + (Mr*(2*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*cos(phi_w + theta_1 + theta_2))*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*cos(phi_w + theta_1) + L1*cos(phi_w + theta_1 + theta_2)) + 2*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*sin(phi_w + theta_1 + theta_2))*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*sin(phi_w + theta_1) + L1*sin(phi_w + theta_1 + theta_2))))/2)*theta_1_dd + (Ir + (Mr*(2*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*cos(phi_w + theta_1 + theta_2))^2 + 2*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*sin(phi_w + theta_1 + theta_2))^2))/2)*theta_2_dd + (Ir + (Mr*(2*R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*cos(phi_w + theta_1 + theta_2)) + 2*R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*sin(phi_w + theta_1 + theta_2))))/2)*theta_3_dd + (Mr*phi_w_d*(2*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*sin(phi_w + theta_1 + theta_2))*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*cos(phi_w + theta_1)*(phi_w_d + theta_1_d)) - 2*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*sin(phi_w + theta_1 + theta_2))*(Xw_d + R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*cos(phi_w + theta_1)*(phi_w_d + theta_1_d))))/2 - (Mr*theta_3_d*(2*R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(Xw_d + R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*cos(phi_w + theta_1)*(phi_w_d + theta_1_d)) - 2*R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*sin(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*sin(phi_w + theta_1)*(phi_w_d + theta_1_d)) + 2*R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*cos(phi_w + theta_1 + theta_2))*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) - 2*R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*sin(phi_w + theta_1 + theta_2))*(phi_w_d + theta_1_d + theta_2_d + theta_3_d)))/2 - Mr*g*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*sin(phi_w + theta_1 + theta_2)) - (Mr*(2*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d))*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*sin(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*sin(phi_w + theta_1)*(phi_w_d + theta_1_d)) - 2*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*sin(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d))*(Xw_d + R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*cos(phi_w + theta_1)*(phi_w_d + theta_1_d))))/2 + (Mr*theta_1_d*(2*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*sin(phi_w + theta_1 + theta_2))*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*cos(phi_w + theta_1)*(phi_w_d + theta_1_d)) - 2*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*sin(phi_w + theta_1 + theta_2))*(Xw_d + R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*cos(phi_w + theta_1)*(phi_w_d + theta_1_d))))/2 + (Mr*theta_2_d*(2*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*cos(phi_w + theta_1 + theta_2))*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*sin(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*sin(phi_w + theta_1)*(phi_w_d + theta_1_d)) - 2*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*sin(phi_w + theta_1 + theta_2))*(Xw_d + R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*cos(phi_w + theta_1)*(phi_w_d + theta_1_d)) + 2*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d))*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*sin(phi_w + theta_1 + theta_2)) - 2*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*cos(phi_w + theta_1 + theta_2))*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*sin(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d))))/2 ;
%eqs_M(5,1) = Mr*R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*Xw_dd + (Ir + (Mr*(2*R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*cos(phi_w + theta_1) + L1*cos(phi_w + theta_1 + theta_2)) + 2*R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*sin(phi_w + theta_1) + L1*sin(phi_w + theta_1 + theta_2))))/2)*phi_w_dd + (Ir + (Mr*(2*R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*cos(phi_w + theta_1) + L1*cos(phi_w + theta_1 + theta_2)) + 2*R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L2*sin(phi_w + theta_1) + L1*sin(phi_w + theta_1 + theta_2))))/2)*theta_1_dd + (Ir + (Mr*(2*R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*cos(phi_w + theta_1 + theta_2)) + 2*R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3) + L1*sin(phi_w + theta_1 + theta_2))))/2)*theta_2_dd + (Ir + (Mr*(2*R^2*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)^2 + 2*R^2*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)^2))/2)*theta_3_dd - (Mr*(2*R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*sin(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*sin(phi_w + theta_1)*(phi_w_d + theta_1_d))*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) - 2*R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d)*(Xw_d + R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*cos(phi_w + theta_1)*(phi_w_d + theta_1_d))))/2 - (Mr*phi_w_d*(2*R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(Xw_d + R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*cos(phi_w + theta_1)*(phi_w_d + theta_1_d)) - 2*R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*cos(phi_w + theta_1)*(phi_w_d + theta_1_d))))/2 - (Mr*theta_1_d*(2*R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(Xw_d + R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*cos(phi_w + theta_1)*(phi_w_d + theta_1_d)) - 2*R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*cos(phi_w + theta_1)*(phi_w_d + theta_1_d))))/2 - (Mr*theta_3_d*(2*R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(Xw_d + R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*cos(phi_w + theta_1)*(phi_w_d + theta_1_d)) - 2*R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*sin(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*sin(phi_w + theta_1)*(phi_w_d + theta_1_d))))/2 - (Mr*theta_2_d*(2*R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(Xw_d + R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*cos(phi_w + theta_1)*(phi_w_d + theta_1_d)) - 2*R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*cos(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d)) + 2*R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*sin(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d)) - 2*R*cos(phi_w - alpha + theta_1 + theta_2 + theta_3)*(R*sin(phi_w - alpha + theta_1 + theta_2 + theta_3)*(phi_w_d + theta_1_d + theta_2_d + theta_3_d) + L1*sin(phi_w + theta_1 + theta_2)*(phi_w_d + theta_1_d + theta_2_d) + L2*sin(phi_w + theta_1)*(phi_w_d + theta_1_d))))/2 - Mr*R*g*sin(phi_w - alpha + theta_1 + theta_2 + theta_3);
M = jacobian(eqs_M, q_dd);

%Solving for C
q_double_temp = M*q_dd; %Something strange happening here
temp_difference = simplify(eqs_temp - q_double_temp); % Original equation - q_dd terms
C(1,1) = (Tau1/R) - temp_difference(1,1);
C(2,1) = 0 - temp_difference(2,1);
C(3,1) = Tau1 - temp_difference(3,1);
C(4,1) = Tau2 - temp_difference(4,1);
C(5,1) = Tau3 - temp_difference(5,1);

%%
%State-space form 
left_side_eq = M\C; 
A_mat_top = [zeros(5,5) eye(5,5)];
A_mat_bottom_left = jacobian(left_side_eq, q);
A_mat_bottom_right = jacobian(left_side_eq, q_d);
A_mat_bottom = [A_mat_bottom_left A_mat_bottom_right];
A_mat = [A_mat_top; A_mat_bottom];

B_mat_top = zeros(5,3);
B_mat_bottom = sym(zeros(5,3));
B_mat_bottom(1,1) = 1/R;
B_mat_bottom(3,1) = 1;
B_mat_bottom(4,2) = 1;
B_mat_bottom(5,3) = 1;
B_mat = [B_mat_top; B_mat_bottom];


%%
%LQR Controller Design

%g = 9.81; % (N/m^2)
%r = .1; % radius of wheel(m)
%m_r = 5.6; % robot mass (kg)
%I_r = .05; % robot moment of inertia (kg*m^2)
%m_w = 5*10^-2; % wheel mass (kg)
%I_w = 5*10^-4; % wheel moment of inertia (kg*m^2)
%z_r_max = .5; % robot max height (m)
%rl_1 = .2; % robot link 1 height (m)
%rl_2 = .2; % robot link 2 height (m)

subs(L1,.2);
subs(L2,.2);
subs(Mw, 5*10^-2);
subs(Mr,5.6);
subs(Mm,.6);
subs(Iw, 5*10^-4);
subs(Ir,.05)
subs(R,.1);
subs(g,9.81);
subs(alpha, 0);

Q = eye(10);
R = eye(3);

K = 0;
%K = lqr(A_mat, B_mat,Q, R);
x = zeros(10,51);
x0 = zeros(10,1);
x(:,1) = x0;
y = zeros(3,50);
for i = [1:51]
    x(:,i+1) = (A_mat-B_mat*K)*x(:,i);
    y(:,i) = x(:,i);
end
%u = -K*x;
%steps = [0:0.2:10.2];
%params = [L1 L2 Mw Mr Mm Iw Ir R g alpha]; % The last variable is alpha for rn
%K = controller(A_mat, B_mat, params, x_initial);










