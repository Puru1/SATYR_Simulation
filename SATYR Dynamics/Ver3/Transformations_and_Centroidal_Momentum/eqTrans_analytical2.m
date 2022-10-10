%% Variable declaration and setup
clearvars -except angle_of_linearization theta2_num_rad
syms xW theta1 theta2 theta3 real
syms dxW dtheta1 dtheta2 dtheta3 real
syms ddxW ddtheta1 ddtheta2 ddtheta3 real
syms theta_t zR real 
syms dtheta_t dzR real
syms L1 L2 L3 real
syms g mW mR IW IR mCM1 mCM2 m_motor R real

q = [xW;theta1;theta2;theta3];
dq = [dxW;dtheta1;dtheta2;dtheta3];
ddq = [ddxW;ddtheta1;ddtheta2;ddtheta3];

%Finding theta1_num and theta2_num
global valM valL
valM = [.5 .1 .1 15];
valL = [.5 .5 .1];

use_joints = false;
theta1_num = pi/18; %angle of linearization
theta2_num = findTheta2(theta1_num, use_joints);
theta3_num = 0;


%% HOMOGENEOUS TRANSFORMS & FRAME INIT.
%Rotation axis for each joint
uy = [0; 1; 0]; %Wheel, knee and hip rotation around y axis

%Rotation and homogeneous transformation matrices for each joint
%Frames from base to end-effector: rotation followed by translation
%Joint 0 (wheel), %Joint 1 (knee), %Joint 2 (Hip), %Joint 3 (Robot)

%Base(B) -> Wheel(0)
RB0 = [[cos(theta1) 0 sin(theta1)]; 
       [0 1 0];
       [-sin(theta1) 0 cos(theta1)]];
vB0 = [xW;0;0];
HTMB0 = [[RB0, vB0]
         [0 0 0 1]];
     
%Wheel(0) -> CM1 
R0cm1 = eye(3);
v0cm1 = [0;0;L1/2]; %  Translation from frame wheel to frame knee (written in frame wheel)
HTM0cm1 = [[R0cm1 v0cm1] 
         [0 0 0 1]];
HTMBcm1 = HTMB0*HTM0cm1;
     
%Wheel(0) -> Knee(1) 
R01 = [[cos(theta2) 0 sin(theta2)]; 
       [0 1 0];
       [-sin(theta2) 0 cos(theta2)]];
RB1 = RB0*R01;
v01 = [0;0;L1]; %  Translation from frame wheel to frame knee (written in frame wheel)
HTM01 = [[R01 v01] 
         [0 0 0 1]];
HTMB1 = simplify(HTMB0*HTM01);

%Knee -> CM2
R1cm2 = eye(3);
v1cm2 = [0;0;L2/2]; %  Translation from frame wheel to frame knee (written in frame wheel)
HTM1cm2 = [[R1cm2 v1cm2] 
         [0 0 0 1]];
HTMBcm2 = HTMB1*HTM1cm2;
     
%Knee(1) -> Hip(2)   
R12 = [[cos(theta3) 0 sin(theta3)];
       [0 1 0];
       [-sin(theta3) 0 cos(theta3)];];
RB2 = RB1*R12;
v12 = [0; 0; L2];%Translation from frame knee to frame hip (written in frame knee)
HTM12 = [[R12 v12] 
         [0 0 0 1]];
HTM02 = simplify(HTM01*HTM12); %Wheel -> hip
HTMB2 = simplify(HTMB0*HTM02); %Base -> hip

%Hip(2) -> Robot(3)
R23 = eye(3);
RB3 = RB2*R23;
v23 = [0; 0; L3];%Translation from frame hip to frame robot (written in frame hip)
HTM23 = [[R23 v23]    
         [0 0 0 1]];
HTM03 = simplify(HTM02*HTM23); %Wheel -> Robot
HTMB3 = simplify(HTMB0*HTM03); %Base -> Robot

Rot = {RB0 RB1 RB2 RB3};   

%Position of coordinate frames and CoM origin in respect to frame 0
PosW = HTMB0(1:3,4);
PosCM1 = HTMBcm1(1:3,4);
PosK = HTMB1(1:3,4);
PosCM2 = HTMBcm2(1:3,4);
PosH = HTMB2(1:3,4);
PosR = HTMB3(1:3,4); %End-effector origin
Pos = [PosW PosCM1 PosCM2 PosR]; %{VaseWheel, Knee, Hip, Robot}

%% Finding the linear transform from x to y via y = Tx + b

X = PosR(1) - PosW(1);
X_d = simplify(jacobian(X,[theta1 theta2 theta3])*[dtheta1 dtheta2 dtheta3]');

zR = PosR(3) - PosW(3);
dzR = simplify(jacobian(X,[theta1 theta2 theta3])*[dtheta1 dtheta2 dtheta3]');
%dzR = -dtheta1*L1*sin(theta1) - (dtheta1+dtheta2)*L2*sin(theta1+theta2) - (dtheta1+dtheta2+dtheta3)*L3*sin(theta1+theta2+theta3);
%ddzR = (-ddtheta1*L1*sin(theta1) - (dtheta1^2)*L1*cos(theta1)) + (-(ddtheta1 + ddtheta2)*L2*sin(theta1+theta2) - (dtheta1^2 + dtheta2^2)*L2*cos(theta1+theta2)) + (-(ddtheta1 + ddtheta2 + ddtheta3)*L3*sin(theta1+theta2+theta3) - (dtheta1^2 + dtheta2^2 + dtheta3^2)*L3*cos(theta1 + theta2 + theta3));

Y = 1/zR;
%Y_d =  (dtheta1*L1*sin(theta1) + (dtheta1+dtheta2)*L2*sin(theta1+theta2) + (dtheta1+dtheta2+dtheta3)*L3*sin(theta1+theta2+theta3))*(Y^2);
Y_d = -dzR/(zR^2);

theta_t = atan(X*Y);
dtheta_t = (1/(1+(X*Y)^2))*(X_d*Y + X*Y_d);

%First order approximation for theta_t
theta_t_constantTerm = simplify(subs(theta_t,[theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 L1 L2 L3],[theta1_num theta2_num 0 0 0 0 .5 .5 .1]));
theta_t_derivativeTerm = jacobian(theta_t,[q; dq]);
theta_t_derivativeTerm = simplify(subs(theta_t_derivativeTerm,[theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 L1 L2 L3],[theta1_num theta2_num 0 0 0 0 .5 .5 .1]));
%uncomment to see single line equation% theta_t_linear = simplify(theta_t_constantTerm + theta_t_derivativeTerm*[q; dq]);
%theta_t_linear = simplify(subs(theta_t_linear,[L1 L2 L3],[.5 .5 .1]))

%First order approximation for zR
zR_constantTerm = simplify(subs(zR,[theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 L1 L2 L3],[theta1_num theta2_num 0 0 0 0 .5 .5 .1]));
zR_derivativeTerm = jacobian(zR,[q; dq]);
zR_derivativeTerm = simplify(subs(zR_derivativeTerm,[theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 L1 L2 L3],[theta1_num theta2_num 0 0 0 0 .5 .5 .1]));
%uncomment to see single line equation% zR_linear = simplify(zR_constantTerm + zR_derivativeTerm*[q; dq]);
%zR_linear = simplify(subs(zR_linear,[L1 L2 L3],[.5 .5 .1]))

%First order approximation for dtheta_t
dtheta_t_constantTerm = simplify(subs(dtheta_t,[theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 L1 L2 L3],[theta1_num theta2_num 0 0 0 0 .5 .5 .1]));
dtheta_t_derivativeTerm = jacobian(dtheta_t,[q; dq]);
dtheta_t_derivativeTerm = simplify(subs(dtheta_t_derivativeTerm,[theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 L1 L2 L3],[theta1_num theta2_num 0 0 0 0 .5 .5 .1]));


%First order approximation for dzR
dzR_constantTerm = simplify(subs(dzR,[theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 L1 L2 L3],[theta1_num theta2_num 0 0 0 0 .5 .5 .1]));
dzR_derivativeTerm = jacobian(dzR,[q; dq]);
dzR_derivativeTerm = simplify(subs(dzR_derivativeTerm,[theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 L1 L2 L3],[theta1_num theta2_num 0 0 0 0 .5 .5 .1]));

%Following the following convention of the 'x' matrix in y = Tx + b: [xW theta1 theta2 theta3 dxW dtheta1 dtheta2 dtheta3]'
xW_row = [1 0 0 0 0 0 0 0];
phi_row = [0 1 -1 1 0 0 0 0]; % Note phi = theta1 - theta2 + theta3
dxW_row = [0 0 0 0 1 0 0 0];
dphi_row = [0 0 0 0 0 1 -1 1];

%Placing the Transformation matrix 'T' together
T_mat = [xW_row; theta_t_derivativeTerm; zR_derivativeTerm; phi_row; dxW_row; dtheta_t_derivativeTerm; dzR_derivativeTerm; dphi_row];
T_mat = vpa(simplify(T_mat));
%Placing the constant 'b' matrix together in y = Tx + b
T_constTerms_mat = [0; theta_t_constantTerm; zR_constantTerm; pi/2; 0; dtheta_t_constantTerm; dzR_constantTerm; 0];
T_constTerms_mat = vpa(simplify(T_constTerms_mat));

%% Finding the centroidal momentum matrix
dxCM1 = simplify(jacobian(PosCM1(1),[theta1 theta2 theta3])*[dtheta1 dtheta2 dtheta3]');
dxCM2 = simplify(jacobian(PosCM2(1),[theta1 theta2 theta3])*[dtheta1 dtheta2 dtheta3]');
dxR = simplify(jacobian(PosR(1),[theta1 theta2 theta3])*[dtheta1 dtheta2 dtheta3]');

dzCM1 = simplify(jacobian(PosCM1(3),[theta1 theta2 theta3])*[dtheta1 dtheta2 dtheta3]');
dzCM2 = simplify(jacobian(PosCM2(3),[theta1 theta2 theta3])*[dtheta1 dtheta2 dtheta3]');
dzR = simplify(jacobian(PosR(3),[theta1 theta2 theta3])*[dtheta1 dtheta2 dtheta3]');

plx = simplify((mW*dxW) + (mCM1*dxCM1) + (mCM2*dxCM2) + (mR*dxR));
ply = 0;
plz = simplify((mCM1*dzCM1) + (mCM2*dzCM2) + mR*dzR);
p = [plx;ply;plz];

Acmm = jacobian(p, dq);
dAcmm(1,:) = simplify(jacobian(Acmm(1,:),[theta1 theta2 theta3])*[dtheta1 dtheta2 dtheta3]')';
dAcmm(2,:) = simplify(jacobian(Acmm(2,:),[theta1 theta2 theta3])*[dtheta1 dtheta2 dtheta3]')';
dAcmm(3,:) = simplify(jacobian(Acmm(3,:),[theta1 theta2 theta3])*[dtheta1 dtheta2 dtheta3]')';

%% EXTERNAL FORCES CALCULATIONS 
% Linear momentum 
%Fext represents the x,y,and z forces acting on the robot      
Fext = Acmm*ddq + dAcmm*dq - [0;0;(mR*g)];
Fext_z = Fext(3);