tic
clear all
%% Sym variable def
syms X1 T1 t2 t3 real
syms X1dot T1dot t2dot t3dot real
syms X1ddot T1ddot t2ddot t3ddot real
syms L1 L2 L3 mw mm m1 m2 m3 mr real
syms J g Rw Rm m_delta real
syms tau1 tau2 tau3 real;
vg = [0 0 -g]';

%% Rigid Body Generation
N_mass = 6;  % Number of mass
rb(1:N_mass) = RB();

% link decl
N_link = 4;     % 1-wheel, 2-shank, 3-thigh, 4-robot

% pos decl
N_pos = 6;      
J1_ = 1;    CM12_ = 2;  J2_ = 3;    CM23_ = 4;  J3_ = 5;    JR_ =6;  % XX_ = 7  ...add here..

%% Rigid Body Prep - 1
% Inertia
IW = .5*mw*Rw^2; IMF = .5*mm*Rm^2; IL1 = m1*L1^2/12;
IL2 = m2*L2^2/12; IMH = mm*Rm^2; IR  = 1/12*mr*(2*L3)^2;

% following table is needed to build rb(rigidbody) objects
            %Mass numbering, you can add more at the end
            MW_ = 1;    MMF_ =2;    ML1_ =3;    ML2_ = 4;   MMH_ = 5;   MR_ = 6;
m =         [mw,        mm,         m1,         m2,         2*mm,       mr  ];
idx_pos =   [J1_,       J1_,        CM12_,      CM23_,      J3_,        JR_ ];
idx_link =  [1          2           2           3           3           4   ];
iner =      [IW,        IMF,        IL1,        IL2,        IMH,        IR  ];

idx_pos_cv      = num2cell(idx_pos);
idx_link_cv     = num2cell(idx_link);   % prep for feed 
m_cv            = num2cell(m);
iner_cv         = num2cell(iner);

[rb.idx_pos]    = idx_pos_cv{1,:};      % feed properties to RB
[rb.idx_link]   = idx_link_cv{1,:};
[rb.mass]       = m_cv{1,:};
[rb.iner]       = iner_cv{1,:};
%% Homogeneous Transformations
% You need to manually build HT from stick figures
% For a mass of interest, MX
% 1. determine if the MX is the main link or branched mass
% 2-1. if i'th linkage) 
%      - find HT from i-1th' origin to i'th origin
% 2-2. if branched)
%      - find HT from its root 
% 2. obtain absolute HT from base to X        
% 3. Put TbX into T(:,:,MX_)                  

T = sym(zeros(4,4, N_pos));
T(:,:,J1_)  = ht(roty(T1), [X1 0 0]);       % rel/abs
T(:,:,J2_)  = ht(roty(t2), [0 0 L1]);       % rel
T(:,:,J3_)  = ht(roty(t3), [0 0 L2]);       % rel
T(:,:,JR_)  = ht(eye(3), [0 0 L3]);         % rel
T(:,:,CM12_)= ht(eye(3), [0 0 .5*L1]);    % rel
T(:,:,CM23_)= ht(eye(3), [0 0 .5*L2]);    % rel
% add here ...
T = simplify(T);

HT = sym(zeros(4,4, N_pos));
% main link joint
HT(:,:,J1_)     = T(:,:,J1_);
HT(:,:,J2_)     = T(:,:,J1_)*T(:,:,J2_);
HT(:,:,J3_)     = T(:,:,J1_)*T(:,:,J2_)*T(:,:,J3_);
% branches
HT(:,:,CM12_)   = T(:,:,J1_)*T(:,:,CM12_);
HT(:,:,CM23_)   = T(:,:,J1_)*T(:,:,J2_)*T(:,:,CM23_);
HT(:,:,JR_)     = T(:,:,J1_)*T(:,:,J2_)*T(:,:,J3_)*T(:,:,JR_);
%add here ...
HT = simplify(HT);

%% Orientations 
ang = sym(zeros(3,N_link));
% defined according to link No.
%           link1   link2   link3   link4     ... you can add at the end /
ang(2,:) =  [X1/Rw, T1,     T1+t2,  T1+t2+t3];

%% Positions
p = sym(zeros(3,N_pos));        % position from base
for i=1:N_pos
    p(:,i) = getDisp(HT(:,:,i));
end

%% Rigid Body - 2
ori = ang(:, idx_link);      % build orientation of mass
pos = p(:,idx_pos);      % build positions of mass 

ori_conv    = num2cell(ori,1);   % prep for feeding
pos_conv    = num2cell(pos,1);  %

[rb.pos]    = pos_conv{1,:};
[rb.ori]    = ori_conv{1,:};

%% Screws
s = sym(zeros(6, N_mass));
s(1:3,:) = [rb.pos];            % build screw
s(4:6,:) = [rb.ori];

%% States - Generalized Coordinate
q = [X1, T1, t2, t3]';
qdot = [X1dot, T1dot, t2dot, t3dot]';
qddot = [X1ddot, T1ddot, t2ddot, t3ddot]';

%% Jacobians / Inertial Tensor / System Inertia
% initialize sym matrix
J = sym(zeros(6, length(q),N_mass));
I = sym(zeros(6,6, N_mass));
M = sym(zeros(length(q),length(q)));

for i = 1:N_mass
    J(:,:,i) = jacobian(s(:,i), q);
    I(:,:,i) = [ rb(i).mass*diag([1 1 1]), zeros(3,3) ;
                 zeros(3,3),               [0 0 0;0 rb(i).iner 0; 0 0 0]];
    M(:,:) = M(:,:) + J(:,:,i)'*I(:,:,i)*J(:,:,i);
end
%% System Energy
Kin = .5.*qdot'*M*qdot;
Pot = dot(sum([rb.mass].*[rb.pos],2), [0 0 g]);
Lag = Kin-Pot;

%% System EOM
tau_ext = [-tau1/Rw, tau1, tau2, tau3];                 % right hand side
feedStates = reshape([q,qdot,qddot]', 1, length(q)*3);  % prep for lagrange
Left_EOM = transpose(Lagrange(Lag, feedStates));             % left hand side
H = jacobian(Left_EOM, qddot); 
C = simplify(Left_EOM - H*qddot);
f = inv(H)*(tau_ext'-C);        % EOM

%% Physical Parameters
% symX: array of sym variable
% valX: actual values corresponding to symX
global valM valL 
symM =  [mw,     mm,     m1,     m2,     mr];
valM =  [0.2,    0,    0.15,    0.15,    15];    % simplifed by m1,m2 = 0
symL =  [Rw,     Rm,     L1,     L2,     L3];
valL =  [0.1,    0.04,   .25,    .25,    .08];
symTau= [tau1 tau2 tau3];

%% TESTING MATRIX CONDITIONING
H_test = vpa(subs(H, [symM symL T1 t2 t3], [valM valL pi/18 -0.3079 0]))
[eigVec, eigVal] = eig(H_test);
eigVal_ar = diag(eigVal);
cond = max(eigVal_ar)/min(eigVal_ar)

%% Linearization
% Linear: dXdot = AdX +BdTau;
% Point of Linearization = (Xw , T1, t2, t3 ) = (0, 30, -75, ?)
% first, find t3 that stabilizes by itself
nPrec = 8;
pre_valq_lin = [0, pi/18, 0];  % [xW theta1 theta3] find t2 and augment it

%% Center of Mass Calc for Linearization
%% x position of center of mass = 0
pCM = sum([rb.mass].*[rb.pos],2)/sum([rb.mass]);
eqn = vpa(subs(pCM(1), [q(1:3)', symM, symL], [pre_valq_lin, valM, valL]), nPrec) == 0 ;

use_joints = true;
lin_t2 = comCalculation(pre_valq_lin(2), use_joints); 
%lin_t3 = min(solve(eqn, t3, 'Real',true));   % find t3 that satisfies the given COM condition

%% Updated Point of Linearization
%valq_lin = [pre_valq_lin, lin_t3];          % found t3 is augmented
valq_lin = [pre_valq_lin(1), pre_valq_lin(2), lin_t2, pre_valq_lin(3)];
valqdot_lin = [ 0 0 0 0 ];

%% Torque at the Point of Linearization by Gravity
sym_tau3_lin = cross(mr*(p(:,JR_)-p(:,J3_)), vg);
sym_tau2_lin = cross(mr*(p(:,JR_)-p(:,J2_))+2*mm*(p(:,J3_)-p(:,J2_))+m2*(p(:,CM23_)-p(:,J2_)), vg);
sym_tau1_lin = cross(mr*(p(:,JR_)-p(:,J1_))+2*mm*(p(:,J3_)-p(:,J1_))+m2*(p(:,CM23_)-p(:,J1_))+m1*(p(:,CM12_)-p(:,J1_)), vg);
valTau_lin = vpa(subs([sym_tau1_lin(2), sym_tau2_lin(2), sym_tau3_lin(2)], [q', symM, symL, g], [valq_lin, valM, valL, 9.81]), nPrec); %taking only y component

%% Linearized Model Acquisition
sym_ar =     [q',       qdot',      symM,   symL,   symTau,     g   ];
val_ar = vpa([valq_lin, valqdot_lin,valM,   valL,   -valTau_lin, 9.81], nPrec);

A_raw = jacobian(f, [q; qdot]);
A_lin = vpa(subs(A_raw, sym_ar, val_ar), nPrec);
A_lqr = [ zeros(4,4), eye(4);A_lin];

B_raw = jacobian(f,symTau);
B_lin = vpa(subs(B_raw, sym_ar, val_ar), nPrec);
B_lqr = [ zeros(4,3); B_lin];
%% Graphics
valP = vpa(subs([p,pCM], sym_ar, val_ar), nPrec);
HT_eval = double(vpa(subs(HT, sym_ar, val_ar), nPrec));
arrow_size = 0.0001;
clf
figure(1)
plotcom(double([valP(1,7), valP(2,7), valP(3,7)]), eye(3),.000001,'smooth')
hold on
plot3(valP(1,1:N_pos), valP(2,1:N_pos), valP(3,1:N_pos), '-ko')
plotCoord(HT_eval, arrow_size)
% plot3(valP(1,7), valP(2,7), valP(3,7), 'ko', 'MarkerSize',11)
axis square;
view([0 0])
daspect(ones(3,1))
grid minor

toc

function plotCoord(HT, arrow_size)
sizeVec = size(HT);
N = sizeVec(3)-1;

ux = quiver3(HT(13:16:13+16*N), HT(14:16:14+16*N), HT(15:16:15+16*N), HT(1:16:1+16*N), HT(2:16:2+16*N), HT(3:16:3+16*N), arrow_size, 'LineWidth',2.5);
uy = quiver3(HT(13:16:13+16*N), HT(14:16:14+16*N), HT(15:16:15+16*N), HT(5:16:5+16*N), HT(6:16:6+16*N), HT(7:16:7+16*N), arrow_size, 'LineWidth',2.5);
uz = quiver3(HT(13:16:13+16*N), HT(14:16:14+16*N), HT(15:16:15+16*N), HT(9:16:9+16*N), HT(10:16:10+16*N), HT(11:16:11+16*N), arrow_size, 'LineWidth',2.5);
end
