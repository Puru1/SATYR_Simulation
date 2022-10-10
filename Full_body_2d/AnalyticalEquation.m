%% VARIABLE DECLARATION
clc;
clear all;

syms xW theta1 theta2 theta3 real
syms dxW dtheta1 dtheta2 dtheta3 real
syms ddxW ddtheta1 ddtheta2 ddtheta3 tau1 real
syms g mW mCM IW ICM L_cm L_torso L_elbow L_forearm R real

%theta_com = 0.0523599; %3 degrees
syms theta_com theta2 L1
%States
q = [xW; theta1; theta2; theta3];
dq = [dxW; dtheta1; dtheta2; dtheta2];
ddq = [ddxW; ddtheta1; ddtheta2; ddtheta3];

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
 
%Wheel(0) -> Torso(1)
R01 = [[cos(theta2) 0 sin(theta2)]; 
       [0 1 0];
       [-sin(theta2) 0 cos(theta2)]];
v01 = [0;0;L_torso]; %  Translation from frame wheel to frame knee (written in frame wheel)
RB1 = RB0*R01;
HTM01 = [[R01 v01] 
         [0 0 0 1]];
HTMB1 = HTMB0*HTM01;

%Torso(1) -> Elbow(2)
R12 = [[cos(theta3) 0 sin(theta3)];
       [0 1 0];
       [-sin(theta3) 0 cos(theta3)];];
RB2 = RB1*R12;
v12 = [0; 0; L_elbow];%Translation from frame knee to frame hip (written in frame knee)
HTM12 = [[R12 v12] 
         [0 0 0 1]];
HTM02 = simplify(HTM01*HTM12); %Wheel -> elbow
HTMB2 = simplify(HTMB0*HTM02); %Base -> elbow

%Hip(2) -> End-effector
R23 = eye(3);
RB3 = RB2*R23;
v23 = [0; 0; L_forearm];%Translation from frame hip to frame robot (written in frame hip)
HTM23 = [[R23 v23]    
         [0 0 0 1]];
HTM03 = simplify(HTM02*HTM23); %Wheel -> end effector
HTMB3 = simplify(HTMB0*HTM03); %Base -> end effector

Rot = {RB0 RBcm RB1};

%Torso


%% JACOBIANS
PosW = HTMB0(1:3,4);
PosCM = HTMBcm(1:3,4);
PosTorso = HTMB1(1:3,4);
Pos = [PosW PosCM PosTorso];

%Mass of bodies
vMass = [mW mCM];

JvW = jacobian(PosW(1:3),q);
JvCM = jacobian(PosCM(1:3),q);
Jv = {JvW JvCM};

% IW = [[.000423   0     0    ];
%       [   0   .00075   0    ];
%       [   0      0  .000423 ]];

% ICM = [[.23368    0     0    ];
%       [   0   .19732   0    ];
%       [   0      0   .06132 ]];
% ICM = [[0    0     0    ];
%       [   0   0  0    ];
%       [   0      0   0]];
  
I = {IW ICM};
% ICM = 0;

w0 = [0;dxW/R;0];
wcm = [0;dtheta1;0];

Jw0 = jacobian(w0,dq); %Wheel
Jwcm = jacobian(wcm,dq); %CM
Jw = {Jw0 Jwcm};

%Manipulator inertia matrix.
M = zeros(2);
for i = 1:2
    M = M + (vMass(i)*Jv{i}'*Jv{i} + Jw{i}'*Rot{i}*I{i}*Rot{i}'*Jw{i});
end

m_list = import_m_list();
write_fcn_m('fnc_PosW.m',{'q','L'},[m_list.q;m_list.L],{PosW,'PosW'});
write_fcn_m('fnc_PosCM.m',{'q','L'},[m_list.q;m_list.L],{PosCM,'PosCM'});

%% KINETIC AND POTENTIAL ENERGY
K = (1/2)*dq'*M*dq;

%Potential Energy
Pw = 0;
Pcm = -mCM*[0 0 -g]*PosCM;
P = Pw+ Pcm;

%Graviational torques vector
G = jacobian(P,q).';

%Simplifying the expressions for each component of the inertia matrix
for i = 1:2
    for j = 1:2
        M(i,j) = simplify(M(i,j));
    end
    G(i,1) = simplify(G(i,1));
end
%% LAGRANGIAN
L = K - P;
dLdqi = transpose(jacobian(L,q));
dLddqi = transpose(jacobian(L,dq));
eqs = simplify(jacobian(dLddqi,q)*dq + jacobian(dLddqi,dq)*ddq - dLdqi);

%Equations of motions H(q)*ddq + C = tau
H = jacobian(dLddqi,dq);
C_temp = jacobian(dLddqi,q);
C = jacobian(dLddqi,q)*dq - dLdqi;
u = [tau1/R; -tau1];
tau = tau1;
H_inv = inv(H);
f = H_inv *(u-C); % The simplify() command and the \ seem to be the problem
%f_num = simplify(vpa(subs(f,[g mW mCM L1 R],[p.g p.mW p.mCM p.L1 p.R])));

%% LINEARIZATION
% p = paramaters
p = getParams();
L = p.L1;
M = [p.mW, p.mCM];
xW = 0;
            % xW  |  theta1  |  dxW   |  dtheta1   
states_lin = [ 0   p.theta1_num   0        0];
% states_lin = [ 0   .03   .25        0];


tau_lin = - vpa(subs(C,[q' dq' g mCM L1],[states_lin p.g p.mCM p.L1]))
                                                                                 
A_var = jacobian(f,[q; dq]);
B_var = jacobian(f,tau);

tic
A = vpa(subs(A_var,[xW theta1 dxW dtheta1 tau1 g mW mCM L1 R IW ICM],[states_lin tau_lin(2) p.g p.mW p.mCM p.L1 p.R 0 0]));
B = vpa(subs(B_var,[xW theta1 dxW dtheta1 tau1 g mW mCM L1 R IW ICM],[states_lin tau_lin(2) p.g p.mW p.mCM p.L1 p.R 0 0]));
toc
%% FUNCTION(S) GENERATION
% write_fcn_m('fnc_PosCM.m',{'q','L1'},[],{PosCM,'PosCM'});
% write_fcn_m('fnc_JvCM.m',{'q','L1'},[],{JvCM,'JvCM'});
% write_fcn_m('fnc_H.m',{'q','Mass','L1'},[],{H,'H'});
% write_fcn_m('fnc_C.m',{'q','dq','Mass','L1'},[],{C,'C'});
% write_fcn_m('fnc_f.m',{'q','dq','Mass','L1'},[],{f,'f'});

write_fcn_m('fnc_A.m',{},[],{A,'A'});
write_fcn_m('fnc_B.m',{},[],{B,'B'});
write_fcn_m('fnc_tauLin.m',{},[],{tau_lin,'tau_lin'});

%% OLD METHOD
% dx1 = dxW + dtheta1*L1*cos(theta1);
% dxLeg1 = dxW + (1/2)*dtheta1*L1*cos(theta1);
% dz1 = -dtheta1*L1*sin(theta1);
% dzLeg1 = -(1/2)*dtheta1*L1*sin(theta1);
% 
% 
% K_wheel = (mW*dxW^2/2 + IW*(dxW/R)^2/2);
% K_robot = mR*(dx1^2 + dz1^2)/2;
% 
% Kin = K_wheel + K_robot;
% 
% Pot_wheel = 0;
% Pot_robot = mR*g*(L1*cos(theta1));
% Pot = Pot_wheel + Pot_robot;
% 
% %Lagrangian
% L = Kin - Pot;
% dLdqi = transpose(jacobian(L,q));
% dLddqi = transpose(jacobian(L,dq));
% eqs = simplify(jacobian(dLddqi,q)*dq + jacobian(dLddqi,dq)*ddq - dLdqi);
% 
% %Equations of motions H(q,dq)*ddq + C = tau
% H = jacobian(dLddqi,dq)
% C = simplify(jacobian(dLddqi,q)*dq - dLdqi)
% u = [tau1/R; -tau1];
% tau = tau1;
% %u = [0;0;0;0];
% 
% %Linearization: dX = A*X + B*tau
% H_inv = inv(H);
% %f = simplify(H\(u - C));
% f = H_inv *(u-C) % The simplify() command and the \ seem to be the problem




