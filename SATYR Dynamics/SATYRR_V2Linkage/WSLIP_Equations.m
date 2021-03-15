tic
clc;
clear all;

addpath functions;

syms xW theta1 l_h real %joint positions
syms dxW dtheta1 dl_h real%joint velocities
syms ddxW ddtheta1 ddl_h real%joint accelerations
syms tau1 dL real %torques 
syms g mW mB R Ks L0 real% [gravity, wheel mass, knee mass, hip mass, robot mass, wheel inertia, knee inertia, hip inertia ..., wheel radius]
%Joint state vectors and manip. parameters
q = [xW; l_h; theta1];
dq = [dxW; dl_h; dtheta1];
ddq = [ddxW; ddl_h; ddtheta1];

qVec = [xW l_h theta1];
dqVec = [dxW dl_h dtheta1];

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
     
%Wheel(0) -> CMB(R)
R0cmb = eye(3);
v0cmb = [0;0;l_h]; %  Translation from frame wheel to frame knee (written in frame wheel)
R0BR = RB0 * R0cmb;
HTM0cmb = [[R0cmb v0cmb] 
         [0 0 0 1]];
HTMBcmb = HTMB0*HTM0cmb;

Rot = {RB0 R0BR};  % IS THIS CORRECT ??
%% JACOBIANS 
%Position of coordinate frames and CoM origin in respect to frame 0
PosW = HTMB0(1:3,4);
PosB = HTMBcmb(1:3,4);

%Mass of bodies
vMass = [mW mB];

JvW = simplify(jacobian(PosW(1:3),q));
JvCMB = simplify(jacobian(PosB(1:3),q));
Jv = {JvW JvCMB}; %{Jv_wheel, JvRobot)


IW = [[.000423   0     0    ];
      [   0   .00075   0    ];
      [   0      0  .000423 ]];
  
IB = [[.23368    0     0    ];
      [   0   .19732   0    ];
      [   0      0   .06132 ]];

I = {IW IB};
  
%Angular component of the Jacobian for each rigid-body
w0 = [0;dxW/R;0];
wB = [0;dtheta1;0];
Jw0 = jacobian(w0,dq); %Wheel
JwB = jacobian(wB,dq); %CM1

Jw = {Jw0 JwB}; %{Jw_wheel, Jw_robot}

%Manipulator inertia matrix.
M = zeros(3);
for i = 1:2
    M = M + (vMass(i)*Jv{i}'*Jv{i} + Jw{i}'*Rot{i}*I{i}*Rot{i}'*Jw{i});
end

%m_list = import_m_list();

%% KINETIC AND POTENTIAL ENERGY
K = (1/2)*dq'*M*dq;

%Potential Energy
Pw = 0;
Pb = mB*[0 0 g]*PosB;
Pspring = (1/2) * Ks*(L0 - l_h)^2;
P = Pw+ Pb + Pspring;

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
H = simplify(jacobian(dLddqi,dq));
C = simplify(jacobian(dLddqi,q)*dq - dLdqi);

u = [tau1/R; Ks*dL;-tau1]; % Reason behind inversions has to do with us building the model ground up

H_inv = inv(H);
f = simplify(H_inv *(u-C)); % The simplify() command and the \ seem to be the problem

%% LINEARIZATION

% Calculate and show linearization 
p = getParams();
L = [p.valL.L1,p.valL.L2,p.valL.L3];
M = [p.valM.cm1,p.valM.cm2,p.valM.mB];
xW = 0;

L_total = sum(L);
M_total = sum(M);
l_h_lin = L_total - (M_total*9.81)/p.Ks;

            %  xW  |  l_h   |  theta1    |   dxW   |   dl_h  |  dtheta1  | 
states_lin = [  0    l_h_lin       0          0         0        0     ];
tau_lin = vpa(subs(C,[qVec dqVec g mB Ks L0],[states_lin -p.g M_total p.Ks L_total]));

%% STATE SPACE
tau = [tau1;dL];
tauVec = [tau1 dL];
A_var = jacobian(f,[q; dq]);
B_var = jacobian(f,tau);
A = vpa(subs(A_var,[qVec dqVec tauVec g mW mB Ks L0 R], [states_lin, tau_lin(3), tau_lin(2),-p.g, ...
                                                    p.valM.mW, p.valM.mB, p.Ks, L_total, p.R]));
B = vpa(subs(B_var,[qVec dqVec tauVec g mW mB Ks L0 R], [states_lin, tau_lin(3), tau_lin(2),-p.g, ...
                                                    p.valM.mW, p.valM.mB, p.Ks, L_total, p.R]));
%%
m_list = import_m_list();
write_fcn_m('fnc_A.m',{},[],{A,'A'});
write_fcn_m('fnc_B.m',{},[],{B,'B'});
write_fcn_m('fnc_tauLin.m',{},[],{tau_lin,'tau_lin'});

write_fcn_m('fnc_H.m',{'q','L','vMass','Ks'},[m_list.q;m_list.L;m_list.M;m_list.p],{H,'H'});
write_fcn_m('fnc_C.m',{'q','dq','L','vMass','g','Ks'},[m_list.q;m_list.dq;m_list.L;m_list.M;m_list.p],{C,'C'});
