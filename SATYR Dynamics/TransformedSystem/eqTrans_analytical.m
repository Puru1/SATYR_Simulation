%% Variable declaration and setup
clearvars -except angle_of_linearization theta2_num_rad
syms xW theta1 theta2 theta3 
syms dxW dtheta1 dtheta2 dtheta3
syms ddxW ddtheta1 ddtheta2 ddtheta3
syms theta_t zR
syms dtheta_t dzR
syms L1 L2 L3
syms g mW mR IW IR m_leg1 m_leg2 m_motor R 

q = [xW;theta1;theta2;theta3];
dq = [dxW;dtheta1;dtheta2;dtheta3];
ddq = [ddxW;ddtheta1;ddtheta2;ddtheta3];

%% Finding the linear transform from x to y via y = Tx + b

X = L1*sin(theta1) + L2*sin(theta1+theta2) + L3*sin(theta1+theta2+theta3);
X_d = dtheta1*L1*cos(theta1) + (dtheta1+dtheta2)*L2*cos(theta1+theta2) + (dtheta1+dtheta2+dtheta3)*L3*cos(theta1+theta2+theta3);

zR = L1*cos(theta1) + L2*cos(theta1+theta2) + L3*cos(theta1+theta2+theta3);
dzR = -dtheta1*L1*sin(theta1) - (dtheta1+dtheta2)*L2*sin(theta1+theta2) - (dtheta1+dtheta2+dtheta3)*L3*sin(theta1+theta2+theta3);
ddzR = (-ddtheta1*L1*sin(theta1) - (dtheta1^2)*L1*cos(theta1)) + (-(ddtheta1 + ddtheta2)*L2*sin(theta1+theta2) - (dtheta1^2 + dtheta2^2)*L2*cos(theta1+theta2)) + (-(ddtheta1 + ddtheta2 + ddtheta3)*L3*sin(theta1+theta2+theta3) - (dtheta1^2 + dtheta2^2 + dtheta3^2)*L3*cos(theta1 + theta2 + theta3));

Y = 1/zR;
%Y_d =  (dtheta1*L1*sin(theta1) + (dtheta1+dtheta2)*L2*sin(theta1+theta2) + (dtheta1+dtheta2+dtheta3)*L3*sin(theta1+theta2+theta3))*(Y^2);
Y_d = -dzR/(zR^2);

theta_t = atan(X*Y);
dtheta_t = (1/(1+(X*Y)^2))*(X_d*Y + X*Y_d);

%Angle to linearize the following terms around
angle_of_linearization = pi/18;

%First order approximation for theta_t
theta_t_constantTerm = simplify(subs(theta_t,[theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 L1 L2 L3],[angle_of_linearization -angle_of_linearization 0 0 0 0 .5 .5 .1]));
theta_t_derivativeTerm = jacobian(theta_t,[q; dq]);
theta_t_derivativeTerm = simplify(subs(theta_t_derivativeTerm,[theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 L1 L2 L3],[angle_of_linearization -angle_of_linearization 0 0 0 0 .5 .5 .1]));
%uncomment to see single line equation% theta_t_linear = simplify(theta_t_constantTerm + theta_t_derivativeTerm*[q; dq]);
%theta_t_linear = simplify(subs(theta_t_linear,[L1 L2 L3],[.5 .5 .1]))

%First order approximation for zR
zR_constantTerm = simplify(subs(zR,[theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 L1 L2 L3],[angle_of_linearization -angle_of_linearization 0 0 0 0 .5 .5 .1]));
zR_derivativeTerm = jacobian(zR,[q; dq]);
zR_derivativeTerm = simplify(subs(zR_derivativeTerm,[theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 L1 L2 L3],[angle_of_linearization -angle_of_linearization 0 0 0 0 .5 .5 .1]));
%uncomment to see single line equation% zR_linear = simplify(zR_constantTerm + zR_derivativeTerm*[q; dq]);
%zR_linear = simplify(subs(zR_linear,[L1 L2 L3],[.5 .5 .1]))

%First order approximation for dtheta_t
dtheta_t_constantTerm = simplify(subs(dtheta_t,[theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 L1 L2 L3],[angle_of_linearization -angle_of_linearization 0 0 0 0 .5 .5 .1]));
dtheta_t_derivativeTerm = jacobian(dtheta_t,[q; dq]);
dtheta_t_derivativeTerm = simplify(subs(dtheta_t_derivativeTerm,[theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 L1 L2 L3],[angle_of_linearization -angle_of_linearization 0 0 0 0 .5 .5 .1]));


%First order approximation for dzR
dzR_constantTerm = simplify(subs(dzR,[theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 L1 L2 L3],[angle_of_linearization -angle_of_linearization 0 0 0 0 .5 .5 .1]));
dzR_derivativeTerm = jacobian(dzR,[q; dq]);
dzR_derivativeTerm = simplify(subs(dzR_derivativeTerm,[theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 L1 L2 L3],[angle_of_linearization -angle_of_linearization 0 0 0 0 .5 .5 .1]));

%Following the following convention of the 'x' matrix in y = Tx + b: [xW theta1 theta2 theta3 dxW dtheta1 dtheta2 dtheta3]'
xW_row = [1 0 0 0 0 0 0 0];
phi_row = [0 1 -1 1 0 0 0 0]; % Note phi = theta1 - theta2 + theta3
dxW_row = [0 0 0 0 1 0 0 0];
dphi_row = [0 0 0 0 0 1 -1 1];

%Placing the Transformation matrix 'T' together
T_mat = [xW_row; theta_t_derivativeTerm; zR_derivativeTerm; phi_row; dxW_row; dtheta_t_derivativeTerm; dzR_derivativeTerm; dphi_row];

%Placing the constant 'b' matrix together in y = Tx + b
T_constTerms_mat = [0; theta_t_constantTerm; zR_constantTerm; pi/2; 0; dtheta_t_constantTerm; dzR_constantTerm; 0];

%% Finding the centroidal momentum matrix
dxR = dxW + dtheta1*L1*cos(theta1) + (dtheta2+dtheta1)*L2*cos(theta2+theta1) + (dtheta3+dtheta2+dtheta1)*L3*cos(theta3+theta2+theta1);
dxLeg1 = dxW + (1/2)*dtheta1*L1*cos(theta1);
dxLeg2 = dxW + dtheta1*L1*cos(theta1) + (1/2)*(dtheta2+dtheta1)*L2*cos(theta2+theta1);
dzLeg1 = -(1/2)*dtheta1*L1*sin(theta1);
dzLeg2 = -dtheta1*L1*sin(theta1) - (1/2)*(dtheta2+dtheta1)*L2*sin(theta2+theta1);

px = (mW*dxW) + (mR*dxR) + (m_leg1*dxLeg1) + (m_leg2*dxLeg2);
py = 0;
pz = (m_leg1*dzLeg1) + (m_leg2*dzLeg2) + mR*dzR;
p_lin = [px;py;pz];

Acmm = jacobian(p_lin, dq);
dAcmm = [[0, -m_leg1*dtheta1*L1*sin(theta1)/2 + m_leg2*(- dtheta1*L1*sin(theta1) - L2*(dtheta1+dtheta2)*sin(theta1+theta2)) + mR*(- dtheta1*L1*sin(theta1) - (dtheta1+dtheta2)*L2*sin(theta1+theta2) - (dtheta1+dtheta2+dtheta3)*L3*sin(theta1+theta2+theta3)), -m_leg2*L2*(dtheta1+dtheta2)*sin(theta1+theta2)/2 + mR*(-(dtheta1+dtheta2)*L2*sin(theta1+theta2) - (dtheta1+dtheta2+dtheta3)*L3*sin(theta1+theta2+theta3)), -mR*(dtheta1+dtheta2+dtheta3)*L3*sin(theta1+theta2+theta3)]
         [0,                                                                                                0                                                                                                                                                 ,                                                                                 0                                                                         ,                               0                           ]
         [0, -m_leg1*dtheta1*L1*cos(theta1)/2 + m_leg2*(- dtheta1*L1*cos(theta1) - L2*(dtheta1+dtheta2)*cos(theta1+theta2)) + mR*(- dtheta1*L1*cos(theta1) - (dtheta1+dtheta2)*L2*cos(theta1+theta2) - (dtheta1+dtheta2+dtheta3)*L3*cos(theta1+theta2+theta3)), -m_leg2*L2*(dtheta1+dtheta2)*cos(theta1+theta2)/2 + mR*(-(dtheta1+dtheta2)*L2*cos(theta1+theta2) - (dtheta1+dtheta2+dtheta3)*L3*cos(theta1+theta2+theta3)), -mR*(dtheta1+dtheta2+dtheta3)*L3*cos(theta1+theta2+theta3)]];
%Fext represents the x,y,and z forces acting on the robot      
Fext = Acmm*ddq + dAcmm*dq - [0;0;(mR*g)];
Fext_z = Fext(3);


%% NUMERICAL SECTION
%Solving ODE for q and dq

global K 
g = 9.81;
mW = .5;
mR = 15;
m_leg1 = .1;
m_leg2 = .1;
m_motor = .2;
L1 = 0.5; 
L2 = 0.5;
h = L1;
L3 = .1;
R = 0.05;
IW = mW*R^2;
IR = mR*(2*L3)^2/12;

%% State Space
A = [[ 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    -(R^2*(10*m_motor + 5*g*mR + g*m_motor))/(5*(mW*R^2 + IW)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      (L3*R^2*g*mR)/(L1*(mW*R^2 + IW)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               (2*L3*R^2*g*mR)/(L1*(mW*R^2 + IW)), 0, 0, 0, 0]
[ 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          (20*IR*L2^2*R^2*m_motor^3 + 2*IR*L2^2*R^2*g*m_motor^3 + 10*IR*L2^2*R^2*mR*m_motor^2 + 110*IR*L3^2*R^2*mR*m_motor^2 + 50*IR*IW*L3^2*mR*m_motor + 20*L2^2*L3^2*R^2*mR*m_motor^3 + 25*IR*IW*L3^2*g*mR^2 + 10*IR*IW*L2^2*g*m_motor^2 + 2*L2^2*L3^2*R^2*g*mR*m_motor^3 + 20*IR*L2*L3*R^2*mR*m_motor^2 + 50*IR*L3^2*R^2*mR*mW*m_motor + 50*IR*IW*L2*L3*mR*m_motor + 10*L2^2*L3^2*R^2*g*mR^2*m_motor^2 + 10*IW*L2^2*L3^2*g*mR*m_motor^2 + 25*IR*L3^2*R^2*g*mR^2*mW + 11*IR*L2^2*R^2*g*mR*m_motor^2 + 5*IR*L2^2*R^2*g*mR^2*m_motor + 11*IR*L3^2*R^2*g*mR*m_motor^2 + 55*IR*L3^2*R^2*g*mR^2*m_motor + 10*IR*L2^2*R^2*g*mW*m_motor^2 + 25*IR*IW*L2*L3*g*mR^2 + 5*IR*IW*L2^2*g*mR*m_motor + 5*IR*IW*L3^2*g*mR*m_motor + 50*IR*L2*L3*R^2*mR*mW*m_motor + 10*L2^2*L3^2*R^2*g*mR*mW*m_motor^2 + 25*IR*L2*L3*R^2*g*mR^2*mW + 2*IR*L2*L3*R^2*g*mR*m_motor^2 + 10*IR*L2*L3*R^2*g*mR^2*m_motor + 5*IR*L2^2*R^2*g*mR*mW*m_motor + 5*IR*L3^2*R^2*g*mR*mW*m_motor + 10*IR*IW*L2*L3*g*mR*m_motor + 10*IR*L2*L3*R^2*g*mR*mW*m_motor)/(5*L1*m_motor*(mW*R^2 + IW)*(IR*L2^2*mR + 11*IR*L3^2*mR + 2*IR*L2^2*m_motor + 2*L2^2*L3^2*mR*m_motor + 2*IR*L2*L3*mR)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    -(20*IR*IW*L1*L2^2*m_motor^2 + 5*IR*IW*L3^3*g*mR^2 + 5*IR*IW*L1*L2^2*g*mR^2 + 5*IR*IW*L1*L3^2*g*mR^2 + 10*IR*IW*L2*L3^2*g*mR^2 + 5*IR*IW*L2^2*L3*g*mR^2 + 2*L2^2*L3^3*R^2*g*mR^2*m_motor^2 + 20*IW*L1*L2^2*L3^2*mR*m_motor^2 + 20*IR*L1*L2^2*R^2*mW*m_motor^2 + 10*IW*L2^2*L3^3*g*mR^2*m_motor + 5*IR*L3^3*R^2*g*mR^2*mW + 11*IR*L3^3*R^2*g*mR^2*m_motor + 10*IR*IW*L1*L2^2*mR*m_motor + 10*IR*IW*L1*L2*L3*g*mR^2 + 10*IR*IW*L1*L2^2*g*mR*m_motor + 10*IR*IW*L2^2*L3*g*mR*m_motor + 20*L1*L2^2*L3^2*R^2*mR*mW*m_motor^2 + 10*L2^2*L3^3*R^2*g*mR^2*mW*m_motor - 10*IW*L1*L2*L3^3*g*mR^2*m_motor + 10*IR*L1*L2^2*R^2*mR*mW*m_motor + 10*IR*IW*L1*L2*L3*mR*m_motor + 10*IW*L1*L2^2*L3^2*g*mR^2*m_motor + 5*IR*L1*L2^2*R^2*g*mR^2*mW + 5*IR*L1*L3^2*R^2*g*mR^2*mW + 10*IR*L2*L3^2*R^2*g*mR^2*mW + 5*IR*L2^2*L3*R^2*g*mR^2*mW + 2*IR*L2*L3^2*R^2*g*mR^2*m_motor + 2*IR*L2^2*L3*R^2*g*mR*m_motor^2 + IR*L2^2*L3*R^2*g*mR^2*m_motor + 10*IR*L1*L2^2*R^2*g*mR*mW*m_motor + 10*IR*L2^2*L3*R^2*g*mR*mW*m_motor + 10*IR*IW*L1*L2*L3*g*mR*m_motor - 10*L1*L2*L3^3*R^2*g*mR^2*mW*m_motor + 10*IR*L1*L2*L3*R^2*mR*mW*m_motor + 10*L1*L2^2*L3^2*R^2*g*mR^2*mW*m_motor + 10*IR*L1*L2*L3*R^2*g*mR^2*mW + 10*IR*L1*L2*L3*R^2*g*mR*mW*m_motor)/(L1^2*m_motor*(mW*R^2 + IW)*(IR*L2^2*mR + 11*IR*L3^2*mR + 2*IR*L2^2*m_motor + 2*L2^2*L3^2*mR*m_motor + 2*IR*L2*L3*mR)),                                                                                                                                                            -(2*L3*g*mR*(5*IR*IW*L2^2*mR + 5*IR*IW*L3^2*mR + 10*IR*IW*L2^2*m_motor + 2*IR*L2^2*R^2*m_motor^2 + 2*L2^2*L3^2*R^2*mR*m_motor^2 + 10*IW*L2^2*L3^2*mR*m_motor + 5*IR*L2^2*R^2*mR*mW + 5*IR*L3^2*R^2*mR*mW + IR*L2^2*R^2*mR*m_motor + 11*IR*L3^2*R^2*mR*m_motor + 10*IR*L2^2*R^2*mW*m_motor + 5*IR*IW*L1*L2*mR + 5*IR*IW*L1*L3*mR + 10*IR*IW*L2*L3*mR + 10*IR*IW*L1*L2*m_motor + 10*L2^2*L3^2*R^2*mR*mW*m_motor - 10*IW*L1*L2*L3^2*mR*m_motor + 5*IR*L1*L2*R^2*mR*mW + 5*IR*L1*L3*R^2*mR*mW + 10*IR*L2*L3*R^2*mR*mW + 2*IR*L2*L3*R^2*mR*m_motor + 10*IR*L1*L2*R^2*mW*m_motor - 10*L1*L2*L3^2*R^2*mR*mW*m_motor))/(L1^2*m_motor*(mW*R^2 + IW)*(IR*L2^2*mR + 11*IR*L3^2*mR + 2*IR*L2^2*m_motor + 2*L2^2*L3^2*mR*m_motor + 2*IR*L2*L3*mR)), 0, 0, 0, 0]
[ 0, -(20*IR*L2^2*R^2*m_motor^3 + 2*IR*L2^2*R^2*g*m_motor^3 + 10*IR*L2^2*R^2*mR*m_motor^2 + 110*IR*L3^2*R^2*mR*m_motor^2 - 10*IR*IW*L1*L2*m_motor^2 + 50*IR*IW*L3^2*mR*m_motor + 20*L2^2*L3^2*R^2*mR*m_motor^3 + 25*IR*IW*L3^2*g*mR^2 + 10*IR*IW*L2^2*g*m_motor^2 + 2*L2^2*L3^2*R^2*g*mR*m_motor^3 - 10*IW*L1*L2*L3^2*mR*m_motor^2 + 20*IR*L2*L3*R^2*mR*m_motor^2 - 10*IR*L1*L2*R^2*mW*m_motor^2 + 50*IR*L3^2*R^2*mR*mW*m_motor + 50*IR*IW*L1*L3*mR*m_motor + 50*IR*IW*L2*L3*mR*m_motor + 10*L2^2*L3^2*R^2*g*mR^2*m_motor^2 + 10*IW*L2^2*L3^2*g*mR*m_motor^2 + 25*IR*L3^2*R^2*g*mR^2*mW + 11*IR*L2^2*R^2*g*mR*m_motor^2 + 5*IR*L2^2*R^2*g*mR^2*m_motor + 11*IR*L3^2*R^2*g*mR*m_motor^2 + 55*IR*L3^2*R^2*g*mR^2*m_motor + 10*IR*L2^2*R^2*g*mW*m_motor^2 + 25*IR*IW*L1*L3*g*mR^2 + 25*IR*IW*L2*L3*g*mR^2 + 10*IR*IW*L1*L2*g*m_motor^2 + 5*IR*IW*L2^2*g*mR*m_motor + 5*IR*IW*L3^2*g*mR*m_motor + 50*IR*L1*L3*R^2*mR*mW*m_motor + 50*IR*L2*L3*R^2*mR*mW*m_motor + 10*L2^2*L3^2*R^2*g*mR*mW*m_motor^2 + 10*IW*L1*L2*L3^2*g*mR*m_motor^2 - 5*IW*L1*L2*L3^2*g*mR^2*m_motor + 25*IR*L1*L3*R^2*g*mR^2*mW + 25*IR*L2*L3*R^2*g*mR^2*mW + 2*IR*L2*L3*R^2*g*mR*m_motor^2 + 10*IR*L2*L3*R^2*g*mR^2*m_motor + 10*IR*L1*L2*R^2*g*mW*m_motor^2 + 5*IR*L2^2*R^2*g*mR*mW*m_motor + 5*IR*L3^2*R^2*g*mR*mW*m_motor + 5*IR*IW*L1*L3*g*mR*m_motor + 10*IR*IW*L2*L3*g*mR*m_motor - 10*L1*L2*L3^2*R^2*mR*mW*m_motor^2 + 10*L1*L2*L3^2*R^2*g*mR*mW*m_motor^2 - 5*L1*L2*L3^2*R^2*g*mR^2*mW*m_motor + 5*IR*L1*L3*R^2*g*mR*mW*m_motor + 10*IR*L2*L3*R^2*g*mR*mW*m_motor)/(5*L1*m_motor*(mW*R^2 + IW)*(IR*L2^2*mR + 11*IR*L3^2*mR + 2*IR*L2^2*m_motor + 2*L2^2*L3^2*mR*m_motor + 2*IR*L2*L3*mR)), (20*IR*IW*L1*L2^2*m_motor^2 + 22*IR*IW*L1^2*L2*m_motor^2 + 5*IR*IW*L3^3*g*mR^2 + 5*IR*IW*L1*L2^2*g*mR^2 + 5*IR*IW*L1^2*L2*g*mR^2 + 10*IR*IW*L1*L3^2*g*mR^2 + 5*IR*IW*L1^2*L3*g*mR^2 + 10*IR*IW*L2*L3^2*g*mR^2 + 5*IR*IW*L2^2*L3*g*mR^2 + 2*L2^2*L3^3*R^2*g*mR^2*m_motor^2 + 20*IW*L1*L2^2*L3^2*mR*m_motor^2 + 22*IW*L1^2*L2*L3^2*mR*m_motor^2 + 20*IR*L1*L2^2*R^2*mW*m_motor^2 + 22*IR*L1^2*L2*R^2*mW*m_motor^2 - 11*IW*L1^2*L3^3*g*mR^2*m_motor + 10*IW*L2^2*L3^3*g*mR^2*m_motor + 5*IR*L3^3*R^2*g*mR^2*mW + 11*IR*L3^3*R^2*g*mR^2*m_motor + 10*IR*IW*L1*L2^2*mR*m_motor + 10*IR*IW*L1^2*L2*mR*m_motor + 15*IR*IW*L1*L2*L3*g*mR^2 + 10*IR*IW*L1*L2^2*g*mR*m_motor + 11*IR*IW*L1^2*L2*g*mR*m_motor + 11*IR*IW*L1^2*L3*g*mR*m_motor + 10*IR*IW*L2^2*L3*g*mR*m_motor + 20*L1*L2^2*L3^2*R^2*mR*mW*m_motor^2 + 22*L1^2*L2*L3^2*R^2*mR*mW*m_motor^2 - 11*L1^2*L3^3*R^2*g*mR^2*mW*m_motor + 10*L2^2*L3^3*R^2*g*mR^2*mW*m_motor + 10*IR*L1*L2^2*R^2*mR*mW*m_motor + 10*IR*L1^2*L2*R^2*mR*mW*m_motor + 10*IR*IW*L1*L2*L3*mR*m_motor + 10*IW*L1*L2^2*L3^2*g*mR^2*m_motor + 9*IW*L1^2*L2*L3^2*g*mR^2*m_motor + 5*IR*L1*L2^2*R^2*g*mR^2*mW + 5*IR*L1^2*L2*R^2*g*mR^2*mW + 10*IR*L1*L3^2*R^2*g*mR^2*mW + 5*IR*L1^2*L3*R^2*g*mR^2*mW + 10*IR*L2*L3^2*R^2*g*mR^2*mW + 5*IR*L2^2*L3*R^2*g*mR^2*mW + 2*IR*L2*L3^2*R^2*g*mR^2*m_motor + 2*IR*L2^2*L3*R^2*g*mR*m_motor^2 + IR*L2^2*L3*R^2*g*mR^2*m_motor + 10*IR*L1*L2^2*R^2*g*mR*mW*m_motor + 11*IR*L1^2*L2*R^2*g*mR*mW*m_motor + 11*IR*L1^2*L3*R^2*g*mR*mW*m_motor + 10*IR*L2^2*L3*R^2*g*mR*mW*m_motor + 20*IR*IW*L1*L2*L3*g*mR*m_motor + 10*IR*L1*L2*L3*R^2*mR*mW*m_motor + 10*L1*L2^2*L3^2*R^2*g*mR^2*mW*m_motor + 9*L1^2*L2*L3^2*R^2*g*mR^2*mW*m_motor + 15*IR*L1*L2*L3*R^2*g*mR^2*mW + 20*IR*L1*L2*L3*R^2*g*mR*mW*m_motor)/(L1^2*m_motor*(mW*R^2 + IW)*(IR*L2^2*mR + 11*IR*L3^2*mR + 2*IR*L2^2*m_motor + 2*L2^2*L3^2*mR*m_motor + 2*IR*L2*L3*mR)), (2*L3*g*mR*(5*IR*IW*L1^2*mR + 5*IR*IW*L2^2*mR + 5*IR*IW*L3^2*mR + 11*IR*IW*L1^2*m_motor + 10*IR*IW*L2^2*m_motor + 2*IR*L2^2*R^2*m_motor^2 + 2*L2^2*L3^2*R^2*mR*m_motor^2 - 11*IW*L1^2*L3^2*mR*m_motor + 10*IW*L2^2*L3^2*mR*m_motor + 5*IR*L1^2*R^2*mR*mW + 5*IR*L2^2*R^2*mR*mW + 5*IR*L3^2*R^2*mR*mW + IR*L2^2*R^2*mR*m_motor + 11*IR*L3^2*R^2*mR*m_motor + 11*IR*L1^2*R^2*mW*m_motor + 10*IR*L2^2*R^2*mW*m_motor + 10*IR*IW*L1*L2*mR + 10*IR*IW*L1*L3*mR + 10*IR*IW*L2*L3*mR + 20*IR*IW*L1*L2*m_motor - 11*L1^2*L3^2*R^2*mR*mW*m_motor + 10*L2^2*L3^2*R^2*mR*mW*m_motor - 2*IW*L1^2*L2*L3*mR*m_motor + 10*IR*L1*L2*R^2*mR*mW + 10*IR*L1*L3*R^2*mR*mW + 10*IR*L2*L3*R^2*mR*mW + 2*IR*L2*L3*R^2*mR*m_motor + 20*IR*L1*L2*R^2*mW*m_motor - 2*L1^2*L2*L3*R^2*mR*mW*m_motor))/(L1^2*m_motor*(mW*R^2 + IW)*(IR*L2^2*mR + 11*IR*L3^2*mR + 2*IR*L2^2*m_motor + 2*L2^2*L3^2*mR*m_motor + 2*IR*L2*L3*mR)), 0, 0, 0, 0]
[ 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      -(L2*L3*mR*(2*L2*m_motor + 2*L3*m_motor + L2*g*mR + L3*g*mR - 2*L3*g*m_motor))/(IR*L2^2*mR + 11*IR*L3^2*mR + 2*IR*L2^2*m_motor + 2*L2^2*L3^2*mR*m_motor + 2*IR*L2*L3*mR),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          -(L3*mR*(2*L1*L2^2*m_motor - L1*L2^2*g*mR - 11*L1*L3^2*g*mR + 10*L2*L3^2*g*mR - 4*L1*L2^2*g*m_motor + 22*L1*L2*L3*m_motor + 8*L1*L2*L3*g*mR))/(L1*(IR*L2^2*mR + 11*IR*L3^2*mR + 2*IR*L2^2*m_motor + 2*L2^2*L3^2*mR*m_motor + 2*IR*L2*L3*mR)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    (2*L3*g*mR*(2*L1*L2^2*mR + 11*L1*L3^2*mR - 10*L2*L3^2*mR + 4*L1*L2^2*m_motor + 3*L1*L2*L3*mR))/(L1*(IR*L2^2*mR + 11*IR*L3^2*mR + 2*IR*L2^2*m_motor + 2*L2^2*L3^2*mR*m_motor + 2*IR*L2*L3*mR)), 0, 0, 0, 0]];

A = [[zeros(4,4) eye(4)]; A];
B = [[                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         (R*(L1 + R))/(L1*(mW*R^2 + IW)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 -R^2/(L1*(mW*R^2 + IW)),                                                                                                                                               0]
[                                                                                                                                                                                                          -(5*IR*IW*L2^2*mR + 5*IR*IW*L3^2*mR + 10*IR*IW*L2^2*m_motor + 2*IR*L2^2*R^2*m_motor^2 + 2*L2^2*L3^2*R^2*mR*m_motor^2 + 2*IR*L1*L2^2*R*m_motor^2 + 10*IW*L2^2*L3^2*mR*m_motor + 5*IR*L2^2*R^2*mR*mW + 5*IR*L3^2*R^2*mR*mW + IR*L2^2*R^2*mR*m_motor + 11*IR*L3^2*R^2*mR*m_motor + 10*IR*L2^2*R^2*mW*m_motor + 10*IR*IW*L2*L3*mR + 2*L1*L2^2*L3^2*R*mR*m_motor^2 + 10*L2^2*L3^2*R^2*mR*mW*m_motor + 10*IR*L2*L3*R^2*mR*mW + IR*L1*L2^2*R*mR*m_motor + 11*IR*L1*L3^2*R*mR*m_motor + 2*IR*L2*L3*R^2*mR*m_motor + 2*IR*L1*L2*L3*R*mR*m_motor)/(L1^2*m_motor*(mW*R^2 + IW)*(IR*L2^2*mR + 11*IR*L3^2*mR + 2*IR*L2^2*m_motor + 2*L2^2*L3^2*mR*m_motor + 2*IR*L2*L3*mR)),                                                                                                                                                                (5*IR*IW*L2^2*mR + 5*IR*IW*L3^2*mR + 10*IR*IW*L2^2*m_motor + 2*IR*L2^2*R^2*m_motor^2 + 2*L2^2*L3^2*R^2*mR*m_motor^2 + 10*IW*L2^2*L3^2*mR*m_motor + 5*IR*L2^2*R^2*mR*mW + 5*IR*L3^2*R^2*mR*mW + IR*L2^2*R^2*mR*m_motor + 11*IR*L3^2*R^2*mR*m_motor + 10*IR*L2^2*R^2*mW*m_motor + 5*IR*IW*L1*L2*mR + 5*IR*IW*L1*L3*mR + 10*IR*IW*L2*L3*mR + 10*IR*IW*L1*L2*m_motor + 10*L2^2*L3^2*R^2*mR*mW*m_motor + 10*IW*L1*L2*L3^2*mR*m_motor + 5*IR*L1*L2*R^2*mR*mW + 5*IR*L1*L3*R^2*mR*mW + 10*IR*L2*L3*R^2*mR*mW + 2*IR*L2*L3*R^2*mR*m_motor + 10*IR*L1*L2*R^2*mW*m_motor + 10*L1*L2*L3^2*R^2*mR*mW*m_motor)/(L1^2*m_motor*(mW*R^2 + IW)*(IR*L2^2*mR + 11*IR*L3^2*mR + 2*IR*L2^2*m_motor + 2*L2^2*L3^2*mR*m_motor + 2*IR*L2*L3*mR)),                                 -(10*L2*L3^2*mR)/(L1*(IR*L2^2*mR + 11*IR*L3^2*mR + 2*IR*L2^2*m_motor + 2*L2^2*L3^2*mR*m_motor + 2*IR*L2*L3*mR))]
[ (5*IR*IW*L2^2*mR + 5*IR*IW*L3^2*mR + 10*IR*IW*L2^2*m_motor + 2*IR*L2^2*R^2*m_motor^2 + 2*L2^2*L3^2*R^2*mR*m_motor^2 + 2*IR*L1*L2^2*R*m_motor^2 + 10*IW*L2^2*L3^2*mR*m_motor + 5*IR*L2^2*R^2*mR*mW + 5*IR*L3^2*R^2*mR*mW + IR*L2^2*R^2*mR*m_motor + 11*IR*L3^2*R^2*mR*m_motor + 10*IR*L2^2*R^2*mW*m_motor + 5*IR*IW*L1*L2*mR + 5*IR*IW*L1*L3*mR + 10*IR*IW*L2*L3*mR + 10*IR*IW*L1*L2*m_motor + 2*L1*L2^2*L3^2*R*mR*m_motor^2 + 10*L2^2*L3^2*R^2*mR*mW*m_motor + 10*IW*L1*L2*L3^2*mR*m_motor + 5*IR*L1*L2*R^2*mR*mW + 5*IR*L1*L3*R^2*mR*mW + 10*IR*L2*L3*R^2*mR*mW + IR*L1*L2^2*R*mR*m_motor + 11*IR*L1*L3^2*R*mR*m_motor + 2*IR*L2*L3*R^2*mR*m_motor + 10*IR*L1*L2*R^2*mW*m_motor + 10*L1*L2*L3^2*R^2*mR*mW*m_motor + 2*IR*L1*L2*L3*R*mR*m_motor)/(L1^2*m_motor*(mW*R^2 + IW)*(IR*L2^2*mR + 11*IR*L3^2*mR + 2*IR*L2^2*m_motor + 2*L2^2*L3^2*mR*m_motor + 2*IR*L2*L3*mR)), -(5*IR*IW*L1^2*mR + 5*IR*IW*L2^2*mR + 5*IR*IW*L3^2*mR + 11*IR*IW*L1^2*m_motor + 10*IR*IW*L2^2*m_motor + 2*IR*L2^2*R^2*m_motor^2 + 2*L2^2*L3^2*R^2*mR*m_motor^2 + 11*IW*L1^2*L3^2*mR*m_motor + 10*IW*L2^2*L3^2*mR*m_motor + 5*IR*L1^2*R^2*mR*mW + 5*IR*L2^2*R^2*mR*mW + 5*IR*L3^2*R^2*mR*mW + IR*L2^2*R^2*mR*m_motor + 11*IR*L3^2*R^2*mR*m_motor + 11*IR*L1^2*R^2*mW*m_motor + 10*IR*L2^2*R^2*mW*m_motor + 10*IR*IW*L1*L2*mR + 10*IR*IW*L1*L3*mR + 10*IR*IW*L2*L3*mR + 20*IR*IW*L1*L2*m_motor + 11*L1^2*L3^2*R^2*mR*mW*m_motor + 10*L2^2*L3^2*R^2*mR*mW*m_motor + 20*IW*L1*L2*L3^2*mR*m_motor + 10*IR*L1*L2*R^2*mR*mW + 10*IR*L1*L3*R^2*mR*mW + 10*IR*L2*L3*R^2*mR*mW + 2*IR*L2*L3*R^2*mR*m_motor + 20*IR*L1*L2*R^2*mW*m_motor + 20*L1*L2*L3^2*R^2*mR*mW*m_motor)/(L1^2*m_motor*(mW*R^2 + IW)*(IR*L2^2*mR + 11*IR*L3^2*mR + 2*IR*L2^2*m_motor + 2*L2^2*L3^2*mR*m_motor + 2*IR*L2*L3*mR)),            (L3*mR*(L1*L2 + 11*L1*L3 + 10*L2*L3))/(L1*(IR*L2^2*mR + 11*IR*L3^2*mR + 2*IR*L2^2*m_motor + 2*L2^2*L3^2*mR*m_motor + 2*IR*L2*L3*mR))]
[                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         -(10*L2*L3^2*mR)/(L1*(IR*L2^2*mR + 11*IR*L3^2*mR + 2*IR*L2^2*m_motor + 2*L2^2*L3^2*mR*m_motor + 2*IR*L2*L3*mR)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    (L3*mR*(L1*L2 + 11*L1*L3 + 10*L2*L3))/(L1*(IR*L2^2*mR + 11*IR*L3^2*mR + 2*IR*L2^2*m_motor + 2*L2^2*L3^2*mR*m_motor + 2*IR*L2*L3*mR)), -(L2^2*mR + 11*L3^2*mR + 2*L2^2*m_motor + 2*L2*L3*mR)/(IR*L2^2*mR + 11*IR*L3^2*mR + 2*IR*L2^2*m_motor + 2*L2^2*L3^2*mR*m_motor + 2*IR*L2*L3*mR)]];
B_top = zeros(4,3);
B = [B_top; B];
w = (g/h)^0.5;
%Qq = diag([1 1/R 1/w 1/(w*Rx)]);
%Qq = 350*eye(8);
Qq = diag([1 1 1 1 10 10 1000 1000]);
Ru = diag([1 1 1]);
K = lqr(A,B,Qq,Ru);

q0 = [0; pi/10; -pi/10; 0; 0; 0; 0; 0];
[T,X] = ode45(@SimpleSegway,[0 10],q0);

xW = X(:,1); 
theta1 = X(:,2); 
theta2 = X(:,3);
theta3 = X(:,4);
dxW = X(:,5); 
dtheta1 = X(:,6); 
dtheta2 = X(:,7);
dtheta3 = X(:,8);

%% Plotting Figures Section
figure(1);
plot(T,theta1);
hold on;
plot(T,theta2);
hold on;
plot(T,theta3);
xlabel('Time (s)');
ylabel('Radians');
legend('\theta_{1}', '\theta_{2}', '\theta_{3}');
title('Plotting q');
xlim([0 1]);


figure(2);
plot(T,dtheta1);
hold on;
plot(T,dtheta2);
hold on;
plot(T,dtheta3);
xlabel('Time (s)');
ylabel('Radians/sec');
name = legend('$\dot{\theta_{1}}$', '$\dot{\theta_{2}}$', '$\dot{\theta_{3}}$');
set(name,'Interpreter','latex');
title('Plotting dq');
xlim([0 1]);
