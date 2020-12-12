%% VARIABLE DECLARATION:
tic
clc;
clear all;

syms xW theta1 theta2 theta3 real %joint positions
syms dxW dtheta1 dtheta2 dtheta3 real%joint velocities
syms ddxW ddtheta1 ddtheta2 ddtheta3 real%joint accelerations
syms tau1 tau2 tau3 real%torques
syms g mW mCM1 mK mCM2 mH mR IW IK IH IR L1 L2 L3 R real% [gravity, wheel mass, knee mass, hip mass, robot mass, wheel inertia, knee inertia, hip inertia ..., wheel radius]
syms I0x I0y I0z I1x I1y I1z I2x I2y I2z I3x I3y I3z real
%Joint state vectors and manip. parameters
q = [xW; theta1; theta2; theta3];
dq = [dxW;dtheta1; dtheta2; dtheta3];
ddq = [ddxW;ddtheta1; ddtheta2; ddtheta3];

%Redundant but follows naming convention in other parts of code :/
qVec = [xW theta1 theta2 theta3];
dqVec = [dxW dtheta1 dtheta2 dtheta3];
tauVec = [tau1 tau2 tau3];

L = [L1 L2 L3];
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
%% JACOBIANS 
%Position of coordinate frames and CoM origin in respect to frame 0
PosW = HTMB0(1:3,4);
PosCM1 = HTMBcm1(1:3,4);
PosK = HTMB1(1:3,4);
PosCM2 = HTMBcm2(1:3,4);
PosH = HTMB2(1:3,4);
PosR = HTMB3(1:3,4); %End-effector origin
Pos = [PosW PosCM1 PosCM2 PosR]; %{VaseWheel, Knee, Hip, Robot}

%Mass of bodies
vMass = [mW mCM1 mCM2 mR];

%Jacobian for each CoM linear velocity
JvW = simplify(jacobian(PosW(1:3),q));
JvCM1 = simplify(jacobian(PosCM1(1:3),q));
JvK = simplify(jacobian(PosK(1:3),q));
JvCM2 = simplify(jacobian(PosCM2(1:3),q));
JvH = simplify(jacobian(PosH(1:3),q));
JvR = simplify(jacobian(PosR(1:3),q));
Jv = {JvW JvCM1 JvCM2 JvR}; %{Jv_wheel, Jv_knee, Jv_hip, Jv_robot)

%Inertia tensor (assume diagonal, generaly not the case)
%IW = diag([0 .0013 0]);
IW = [[.000423   0     0    ];
      [   0   .00075   0    ];
      [   0      0  .000423 ]];
%ICM1 = diag([0 .0021 0]);
ICM1 = [[.0127   0        0     ];
       [   0   .0125   -.002    ];
       [   0      0   .000777   ]];
%ICM2 = diag([0 .0021 0]);
ICM2 = [[.00527   0        0      ];
       [   0   .00482   .00011    ];
       [   0   .00011   .00111    ]];
%IR = diag([0 .05 0]);
IR = [[.23368    0     0    ];
      [   0   .19732   0    ];
      [   0      0   .06132 ]];
%IR = diag([I3x I3y I3z]);
I = {IW ICM1 ICM2 IR}; %{I_wheel, I_knee, I_hip, I_robot}
   
%Angular component of the Jacobian for each rigid-body
w0 = [0;dxW/R;0];
wcm1 = [0;dtheta1;0];
wcm2 = [0;dtheta2;0]+ wcm1;
wR = [0;dtheta3;0]+ wcm2;
Jw0 = jacobian(w0,dq); %Wheel
Jwcm1 = jacobian(wcm1,dq); %CM1
Jwcm2 = jacobian(wcm2,dq); %CM2
JwR = jacobian(wR,dq); %Robot
Jw = {Jw0 Jwcm1 Jwcm2 JwR}; %{Jw_wheel, Jw_knee, Jw_hip, Jw_robot}

%Manipulator inertia matrix.
M = zeros(4);
for i = 1:4
    M = M + (vMass(i)*Jv{i}'*Jv{i} + Jw{i}'*Rot{i}*I{i}*Rot{i}'*Jw{i});
end

%% KINETIC AND POTENTIAL ENERGY
K = (1/2)*dq'*M*dq;

%Potential Energy
Pw = 0;
Pcm1 = -mCM1*[0 0 -g]*PosCM1;
Pcm2 = -mCM2*[0 0 -g]*PosCM2;
PR = -mR*[0 0 -g]*PosR;
P = Pw+ Pcm1 + Pcm2 + PR;

%Graviational torques vector
G = jacobian(P,q).';

%Simplifying the expressions for each component of the inertia matrix
for i = 1:4
    for j = 1:4
        M(i,j) = simplify(M(i,j));
    end
    G(i,1) = simplify(G(i,1));
end

%% LAGRANGIAN
L = K - P;
dLdqi = transpose(jacobian(L,q));
dLddqi = transpose(jacobian(L,dq));
eqs = simplify(jacobian(dLddqi,dq)*ddq + jacobian(dLddqi,q)*dq - dLdqi);

%Equations of motions H(q,dq)*ddq + C = uc
H = jacobian(dLddqi,dq)
C = jacobian(dLddqi,q)*dq - dLdqi
u = [tau1/R; -tau1; -tau2; -tau3];
tau = [tau1;tau2;tau3];
H_inv = inv(H);
f = H_inv *(u-C); % The simplify() command and the \ seem to be the problem


%% CENTROIDAL MOMENTUM MATRIX
dW = [dxW;0;0];
dCM1 = JvCM1*dq;
dCM2 = JvCM2*dq;
dR = JvR*dq;
pl = [dW,dCM1,dCM2,dR]*[mW;mCM1;mCM2;mR];
Acmm = jacobian(pl,dq);
dAcmm(1,:) = simplify(jacobian(Acmm(1,:),q) * dq)';
dAcmm(2,:) = simplify(jacobian(Acmm(2,:),q) * dq)';
dAcmm(3,:) = simplify(jacobian(Acmm(3,:),q) * dq)';
%% FINDING LINEAR TRANSFORM FROM X TO Y (Y = TX + V)
%[T,V] = linTransformStates();

%% LINEARIZATION
% p = paramaters
p = getParams();

            % xW  |  theta1  |  theta2    |    theta3   |   dxW   |  dtheta1  |  dtheta2  |  dtheta3 
states_lin = [0  p.theta1_num p.theta2_num   p.theta3_num    0         0           0          0];

% Method 1 (Coriolis Matrix Sub)
tau_lin = - vpa(subs(C,[qVec dqVec g mR mCM1 mCM2 L1 L2 L3],[states_lin p.g p.valM.mR p.valM.cm1...
                                                   p.valM.cm2 p.valL.L1 p.valL.L2 p.valL.L3]));

% Method 2 (Manual Torque Calc.)
r_wcm1 = PosCM1 - PosW;
r_wcm2 = PosCM2 - PosW;
r_wR = PosR - PosW;

r_kcm2 = PosK - PosCM2;
r_kR = PosK- PosR;

r_hR = PosH - PosR;
                                                                                 
tau1_lin = cross(r_wcm1, mCM1*g_acc) + cross(r_wcm2, mCM2*g_acc) + cross(r_wR, mR*g_acc);
tau2_lin = cross(r_kcm2, mCM2*g_acc) + cross(r_kR, mR*g_acc);
tau3_lin = cross(r_hR, mR*g_acc);

tau1_lin = - vpa(subs(tau1_lin, [theta1 theta2 theta3 g mR mCM1 mCM2 L1 L2 L3],[p.theta1_num p.theta2_num... 
                            p.theta3_num p.g p.valM.mR p.valM.cm1 p.valM.cm2 p.valL.L1 p.valL.L2 p.valL.L3]));
                        
tau2_lin = - vpa(subs(tau2_lin, [theta1 theta2 theta3 g mR mCM1 mCM2 L1 L2 L3],[p.theta1_num p.theta2_num... 
                            p.theta3_num p.g p.valM.mR p.valM.cm1 p.valM.cm2 p.valL.L1 p.valL.L2 p.valL.L3]));

tau3_lin = - vpa(subs(tau3_lin, [theta1 theta2 theta3 g mR mCM1 mCM2 L1 L2 L3],[p.theta1_num p.theta2_num... 
                            p.theta3_num p.g p.valM.mR p.valM.cm1 p.valM.cm2 p.valL.L1 p.valL.L2 p.valL.L3]));
                         
%% STATE SPACE FORM
A_var = jacobian(f,[q; dq]);
%A_lin = matlabFunction(A_var);
A = vpa(subs(A_var,[qVec dqVec tauVec g R mW mCM1 mCM2 mR L1 L2 L3],[0 p.theta1_num p.theta2_num p.theta3_num... 
               0 0 0 0 tau1_lin(2) tau2_lin(2) tau3_lin(2) p.g p.R p.valM.mW p.valM.cm1 p.valM.cm2 p.valM.mR ...
               p.valL.L1 p.valL.L2 p.valL.L3]));
B_var = jacobian(f,tau);
B = vpa(subs(B_var,[qVec dqVec tauVec g R mW mCM1 mCM2 mR L1 L2 L3],[0 p.theta1_num p.theta2_num p.theta3_num... 
               0 0 0 0 tau1_lin(2) tau2_lin(2) tau3_lin(2) p.g p.R p.valM.mW p.valM.cm1 p.valM.cm2 p.valM.mR ...
               p.valL.L1 p.valL.L2 p.valL.L3]));        
           
%% FUNCTION(S) GENERATION
% write_fcn_m('fnc_PosCM1.m',{'q','L'},[],{PosCM1,'PosCM1'});
% write_fcn_m('fnc_PosCM2.m',{'q','L'},[],{PosCM2,'PosCM2'});
% write_fcn_m('fnc_PosR.m',{'q','L'},[],{PosR,'PosR'});
% 
% write_fcn_m('fnc_JvCM1.m',{'q','L'},[],{JvCM1,'JvCM1'});
% write_fcn_m('fnc_JvCM2.m',{'q','L'},[],{JvCM2,'JvCM2'});
% write_fcn_m('fnc_JvR.m',{'q','L'},[],{JvR,'JvR'});
% 
% write_fcn_m('fnc_momentum.m',{'q','dq','Mass','L'},[],{pl,'pl'});
% write_fcn_m('fnc_Acmm.m',{'q','dq','Mass','L'},[],{Acmm,'Acmm'});
% write_fcn_m('fnc_dAcmm.m',{'q','dq','Mass','L'},[],{dAcmm,'dAcmm'});
% 
% 
% write_fcn_m('fcn_Lagrangian.m',{'q','dq','L'},[],{f,'f'});
toc
 
