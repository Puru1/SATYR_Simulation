%% VARIABLE DECLARATION
clc;
clear all;
addpath fnc

syms xW theta1 real
syms dxW dtheta1 real
syms ddxW ddtheta1 tau1 real
syms g mW mCM IW ICM L1 R real

%States
q = [xW; theta1];
dq = [dxW; dtheta1];
ddq = [ddxW; ddtheta1];

%% HOMOGENEOUS TRANSFORM AND FRAME INIT.
%Rotation axis
uy = [0; 1; 0]; 

%Base(B) -> Wheel(0)
RB0 = [[cos(theta1) 0 sin(theta1)]; 
       [0 1 0];
       [-sin(theta1) 0 cos(theta1)]];
vB0 = [xW;0;0];
HTMB0 = [[RB0, vB0]
         [0 0 0 1]];

%Wheel(0) -> CM1 
R0cm = eye(3);
v0cm = [0;0;L1]; %  Translation from frame wheel to frame knee (written in frame wheel)
HTM0cm = [[R0cm v0cm] 
         [0 0 0 1]];
HTMBcm = HTMB0*HTM0cm;

Rot = {RB0 R0cm};
%% JACOBIANS
PosW = HTMB0(1:3,4);
PosCM = HTMBcm(1:3,4);
Pos = [PosW PosCM];

%Mass of bodies
vMass = [mW mCM];

JvW = jacobian(PosW(1:3),q);
JvCM = jacobian(PosCM(1:3),q);
Jv = {JvW JvCM};

IW = [[.000423   0     0    ];
      [   0   .00075   0    ];
      [   0      0  .000423 ]];

ICM = [[.23368    0     0    ];
      [   0   .19732   0    ];
      [   0      0   .06132 ]];

I = {IW ICM};

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

%Equations of motions H(q,dq)*ddq + C = tau
H = jacobian(dLddqi,dq);
C = jacobian(dLddqi,q)*dq - dLdqi;
u = [tau1/R; -tau1];
tau = tau1;
H_inv = inv(H);
f = H_inv *(u-C); % The simplify() command and the \ seem to be the problem

%% LINEARIZATION
% p = paramaters
p = getParams();
            % xW  |  theta1  |  dxW   |  dtheta1   
states_lin = [ 0   p.theta1_num   0        0];

tau_lin = - vpa(subs(C,[q' dq' g mCM L1],[states_lin p.g p.mCM p.L1]))
                                                                                 
A_var = jacobian(f,[q; dq]);
A = vpa(subs(A_var,[xW theta1 dxW dtheta1 tau1 g mW mCM L1 R],[states_lin tau_lin(2) p.g p.mW p.mCM p.L1 p.R]))
B_var = jacobian(f,tau);
B = vpa(subs(B_var,[xW theta1 dxW dtheta1 tau1 g mW mCM L1 R],[states_lin tau_lin(2) p.g p.mW p.mCM p.L1 p.R]))

%% FUNCTION(S) GENERATION
% write_fcn_m('fnc_PosCM.m',{'q','L1'},[],{PosCM,'PosCM'});
% write_fcn_m('fnc_JvCM.m',{'q','L1'},[],{JvCM,'JvCM'});
% write_fcn_m('fnc_H.m',{'q','Mass','L1'},[],{H,'H'});
% write_fcn_m('fnc_C.m',{'q','dq','Mass','L1'},[],{C,'C'});
% write_fcn_m('fnc_f.m',{'q','dq','Mass','L1'},[],{f,'f'});

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




