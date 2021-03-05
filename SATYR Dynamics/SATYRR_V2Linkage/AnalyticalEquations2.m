%% VARIABLE DECLARATION:
tic
clc;
clear all;

addpath functions;

syms xW theta1 theta2 theta3 real %joint positions
syms dxW dtheta1 dtheta2 dtheta3 dL_h real%joint velocities
syms ddxW ddtheta1 ddtheta2 ddtheta3 dL_h real%joint accelerations
syms tau1 tau2 tau3 real%torques
syms g mW mCM1 mK mCM2 mH mR IW IK IH IR L1 L2 L3 R H1 real% [gravity, wheel mass, knee mass, hip mass, robot mass, wheel inertia, knee inertia, hip inertia ..., wheel radius]
syms I0x I0y I0z I1x I1y I1z I2x I2y I2z I3x I3y I3z real
%Joint state vectors and manip. parameters
q = [xW;theta1;theta2];
dq = [dxW;dtheta1; dtheta2];
ddq = [ddxW;ddtheta1; ddtheta2];

theta3 = theta2/2;
dtheta3 = theta2/2;
ddtheta3 = ddtheta2/2;

qVec = [xW theta1 theta2];
dqVec = [dxW dtheta1 dtheta2];
tauVec = [tau1 tau2];

L_CM1 = .2169*L1;
L_CM2 = .9*L2;
L_CMRx = -.01884;
L_CMRy = .07329;

L_CM = [L_CM1; L_CM2; L_CMRx; L_CMRy];
L =[L1;L2;L3;R];


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
v0cm1 = [0;0;L_CM1]; %  Translation from frame wheel to frame knee (written in frame wheel)
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
v1cm2 = [0;0;L_CM2]; %  Translation from frame wheel to frame knee (written in frame wheel)
HTM1cm2 = [[R1cm2 v1cm2] 
         [0 0 0 1]];
HTMBcm2 = HTMB1*HTM1cm2;
     
%Knee(1) -> Hip(2)   
R12 = [[cos(theta2/2) 0 sin(theta2/2)];
       [0 1 0];
       [-sin(theta2/2) 0 cos(theta2/2)];];
RB2 = RB1*R12;
v12 = [0; 0; L2];%Translation from frame knee to frame hip (written in frame knee)
HTM12 = [[R12 v12] 
         [0 0 0 1]];
HTM02 = simplify(HTM01*HTM12); %Wheel -> hip
HTMB2 = simplify(HTMB0*HTM02); %Base -> hip

%Hip(2) -> Robot COM(3)
R2cmR = eye(3);
vcmR = [L_CMRx; 0; L_CMRy];%Translation from frame hip to frame robot (written in frame hip)
HTM2cmR = [[R2cmR vcmR]    
         [0 0 0 1]];
HTMBcmR = HTMB2*HTM2cmR ;

%Hip(2) -> Torso
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
PosR = HTMBcmR(1:3,4);
PosT = HTMB3(1:3,4); % Torso pos
Pos = [PosW PosCM1 PosCM2 PosR PosT]; %{VaseWheel, Knee, Hip, Robot}

%Calculate  CoM
pos_com_robot = (mCM1*PosCM1 + mCM2*PosCM2 + mR*PosR)/(mCM1+mCM2+mR);
E1 = pos_com_robot(3) - H1 == 0; % E1 is the z constraints for height 
E2 = pos_com_robot(1) == xW; % E2 is the x constraint to be above the wheel

%Mass of bodies
vMass = [mW mCM1 mCM2 mR];

%Jacobian for each CoM linear velocity
JvW = simplify(jacobian(PosW(1:3),q));
JvCM1 = simplify(jacobian(PosCM1(1:3),q));
JvK = simplify(jacobian(PosK(1:3),q));
JvCM2 = simplify(jacobian(PosCM2(1:3),q));
JvH = simplify(jacobian(PosH(1:3),q));
JvR = simplify(jacobian(PosR(1:3),q));
JvT = simplify(jacobian(PosT(1:3),q));
Jv = {JvW JvCM1 JvCM2 JvR}; %{Jv_wheel, Jv_knee, Jv_hip, Jv_RobotCoM, Jv_torso)


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
M = zeros(3);
for i = 1:4
    M = M + (vMass(i)*Jv{i}'*Jv{i} + Jw{i}'*Rot{i}*I{i}*Rot{i}'*Jw{i});
end

%CENTROIDAL MOMENTUM MATRIX
dW = [dxW;0;0];
dCM1 = JvCM1*dq; 
dCM2 = JvCM2*dq;
dR = JvR*dq;
pl = [dW,dCM1,dCM2,dR]*[mW;mCM1;mCM2;mR];
Acmm = jacobian(pl,dq);
dAcmm(1,:) = simplify(jacobian(Acmm(1,:),q) * dq)';
dAcmm(2,:) = simplify(jacobian(Acmm(2,:),q) * dq)';
dAcmm(3,:) = simplify(jacobian(Acmm(3,:),q) * dq)';

m_list = import_m_list();

write_fcn_m('fullM_fnc_PosCM1.m',{'q','L'},[m_list.q;m_list.L],{PosCM1,'PosCM1'});
write_fcn_m('fullM_fnc_PosK.m',{'q','L'},[m_list.q;m_list.L],{PosK,'PosK'});
write_fcn_m('fullM_fnc_PosCM2.m',{'q','L'},[m_list.q;m_list.L],{PosCM2,'PosCM2'});
write_fcn_m('fullM_fnc_PosH.m',{'q','L'},[m_list.q;m_list.L],{PosH,'PosH'});
write_fcn_m('fullM_fnc_PosCoM_R.m',{'q','L'},[m_list.q;m_list.L],{PosR,'PosR'});
write_fcn_m('fullM_fnc_PosT.m',{'q','L'},[m_list.q;m_list.L],{PosT,'PosT'});
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
for i = 1:3
    for j = 1:3
        M(i,j) = simplify(M(i,j));
    end
    G(i,1) = simplify(G(i,1));
end

%% LAGRANGIAN
L = K - P;
dLdqi = transpose(jacobian(L,q));
dLddqi = transpose(jacobian(L,dq));
eqs = simplify(jacobian(dLddqi,q)*dq + jacobian(dLddqi,dq)*ddq - dLdqi);

%Equations of motions H(q,dq)*ddq + C = tau
H = jacobian(dLddqi,dq)
C = jacobian(dLddqi,q)*dq - dLdqi
u = [tau1/R; -tau1; -tau2];
% u = [-tau1/R; tau1; tau2; tau3];

tau = [tau1;tau2];
H_inv = inv(H);
f = H_inv *(u-C); % The simplify() command and the \ seem to be the problem
%% LINEARIZATION

% Calculate and show linearization 
p = getParams();
L = [p.valL.L1,p.valL.L2,p.valL.L3];
M = [p.valM.cm1,p.valM.cm2,p.valM.mB];
xW = 0;

p.theta1_num = 0;
p.theta2_num = 0;
p.theta3_num = p.theta2_num/2;

q_vis = [xW, p.theta1_num,p.theta2_num,p.theta3_num];
fSingle = figure(99);
ax=axes('Parent',fSingle);
SATYRR_Visualize(q_vis,L,ax);
g_acc = [0;0;-g];
p.theta1_num + p.theta2_num + p.theta3_num

            % xW  |  theta1  |  theta2    |    theta3   |   dxW   |  dtheta1  |  dtheta2  |  dtheta3 
states_lin = [0  p.theta1_num p.theta2_num   p.theta3_num    0         0           0          0];


r_wcm1 = PosCM1 - PosW;
r_wcm2 = PosCM2 - PosW;
r_wR = PosR - PosW;

r_kcm2 = PosK - PosCM2;
r_kR = PosK - PosR;

r_hR = PosH - PosR;

% Corilois matrix method of calculation of tau
tau_lin = vpa(subs(C,[qVec dqVec g mR mCM1 mCM2 L1 L2 L3],[states_lin -p.g p.valM.mR p.valM.cm1...
                                                   p.valM.cm2 p.valL.L1 p.valL.L2 p.valL.L3]))
 
% Manual calculation of tau                                                                               
tau1_lin = cross(r_wcm1, mCM1*g_acc) + cross(r_wcm2, mCM2*g_acc) + cross(r_wR, mR*g_acc);
tau2_lin = cross(r_kcm2, mCM2*g_acc) + cross(r_kR, mR*g_acc);
tau3_lin = cross(r_hR, mR*g_acc);

tau1_lin = - vpa(subs(tau1_lin, [theta1 theta2 theta3 g mR mCM1 mCM2 L1 L2 L3],[p.theta1_num p.theta2_num... 
                            p.theta3_num -p.g p.valM.mR p.valM.cm1 p.valM.cm2 p.valL.L1 p.valL.L2 p.valL.L3]));
                        
tau2_lin = - vpa(subs(tau2_lin, [theta1 theta2 theta3 g mR mCM1 mCM2 L1 L2 L3],[p.theta1_num p.theta2_num... 
                            p.theta3_num -p.g p.valM.mR p.valM.cm1 p.valM.cm2 p.valL.L1 p.valL.L2 p.valL.L3]));

tau3_lin = - vpa(subs(tau3_lin, [theta1 theta2 theta3 g mR mCM1 mCM2 L1 L2 L3],[p.theta1_num p.theta2_num... 
                            p.theta3_num -p.g p.valM.mR p.valM.cm1 p.valM.cm2 p.valL.L1 p.valL.L2 p.valL.L3]));
                         
%% STATE SPACE FORM
A_var = jacobian(f,[q; dq]);
%A_lin = matlabFun ction(A_var);
A = vpa(subs(A_var,[qVec dqVec tauVec g R mW mCM1 mCM2 mR L1 L2 L3],[0 p.theta1_num p.theta2_num p.theta3_num... 
               0 0 0 0 tau_lin(2) tau_lin(3) tau_lin(4) p.g p.R p.valM.mW p.valM.cm1 p.valM.cm2 p.valM.mR ...
               p.valL.L1 p.valL.L2 p.valL.L3]));
B_var = jacobian(f,tau);
B = vpa(subs(B_var,[qVec dqVec tauVec g R mW mCM1 mCM2 mR L1 L2 L3],[0 p.theta1_num p.theta2_num p.theta3_num... 
               0 0 0 0 tau_lin(2) tau_lin(3) tau_lin(4) p.g p.R p.valM.mW p.valM.cm1 p.valM.cm2 p.valM.mR ...
               p.valL.L1 p.valL.L2 p.valL.L3]));        
toc
 
% Copy the following:
% A Matrix
% B Matrix
% H Matrix
% C Matrix
% tau_lin 
% 

%% FUNCTION(S) GENERATION
write_fcn_m('fnc_A.m',{},[],{A,'A'});
write_fcn_m('fnc_B.m',{},[],{B,'B'});
write_fcn_m('fnc_tauLin.m',{},[],{tau_lin,'tau_lin'});

write_fcn_m('fnc_H.m',{'q','L','vMass'},[m_list.q;m_list.L;m_list.M],{H,'H'});
write_fcn_m('fnc_C.m',{'q','dq','L','vMass','g'},[m_list.q;m_list.dq;m_list.L;m_list.M;m_list.p],{C,'C'});

% write_fcn_m('fnc_HTMBcm1.m',{'q','L'},[],{HTMBcm1,'HTMBcm1'});
% write_fcn_m('fnc_HTMBK.m',{'q','L'},[],{HTMB1,'HTMBK'});
% write_fcn_m('fnc_HTMBH.m',{'q','L'},[],{HTMB2,'HTMBH'});
% write_fcn_m('fnc_HTMBT.m',{'q','L'},[],{HTMB3,'HTMBT'});

% write_fcn_m('fnc_JvCM1.m',{'q','L'},[],{JvCM1,'JvCM1'});
% write_fcn_m('fnc_JvCM2.m',{'q','L'},[],{JvCM2,'JvCM2'});
% write_fcn_m('fnc_JvR.m',{'q','L'},[],{JvR,'JvR'});
% 
write_fcn_m('fnc_momentum.m',{'q','dq','vMass','L'},[m_list.q;m_list.dq;m_list.M;m_list.L],{pl,'pl'});
write_fcn_m('fnc_Acmm.m',{'q','vMass','L'},[m_list.q;m_list.dq;m_list.M;m_list.L],{Acmm,'Acmm'});
write_fcn_m('fnc_dAcmm.m',{'q','dq','vMass','L'},[m_list.q;m_list.dq;m_list.M;m_list.L],{dAcmm,'dAcmm'});
% 
% write_fcn_m('fcn_Lagrangian.m',{'q','dq','L'},[],{f,'f'});