function dX = SimpleSegway(t,X,p)

% Local variables
g = p.g;
mW = p.valM.mW;
mCM1 = p.valM.cm1;
mCM2 = p.valM.cm2;
mB = p.valM.mB;
L1 = p.valL.L1; 
L2 = p.valL.L2;
L3 = p.valL.L3;
R = p.R;
IW = p.IW; %
IK = p.IK;
IH = p.IH;
IR = p.IR;
theta1_lin = 0; % manual entry (change later)

L0 = fnc_PosCoM_R([0,0,0],[p.valL.L1,p.valL.L2,p.valL.L3,p.R]);
L0 = L0(3) - .03;
L = [L0,R];
vMass = [mW,mB];
Ks = p.Ks; % manual entry (change later)

% States expanded 
xW = X(1); 
l_h = X(2);
theta1 = X(3); 
dxW = X(4); 
dl_h = X(5); 
dtheta1 = X(6);

q = [xW,l_h,theta1];
dq = [dxW,dl_h,dtheta1];

% Time indicator
t

% Set reference vectors
M_total = sum([p.valM.mW, p.valM.mB]);
% L_h_lin = L0 - (M_total*9.81)/p.Ks;
L_h_lin = L0;
X_ref = [0;L_h_lin;theta1_lin;0;0;0]; % Theta1 given. Found theta2 via COM calculations for stable point.
Bu_ref = fnc_tauLin();
u_ref = [Bu_ref(3); Bu_ref(2)/p.Ks]; % reconstructing u from Bu_ref

M = fnc_H_red(q,L,vMass,Ks);
C = fnc_C_red(q,dq,L,vMass,g,Ks);

% Calculating conditioning of matrix
% [eigVec, eigVal] = eig(M);
% eigVal_ar = diag(eigVal);
% cond = max(eigVal_ar)/min(eigVal_ar);

%Calculate initial torques
K = fnc_K();
X = X - X_ref;

u = -K*X;

% Ensure we normalize properly around linearization 
u = u + u_ref;

% Solving for accelerations and adding damping
Bu = [u(1)/R; Ks*u(2); -u(1)]; 

ddq = M\(Bu - C);

dX = [[dxW; dl_h; dtheta1]; [ddq(1); ddq(2); ddq(3)]];

end