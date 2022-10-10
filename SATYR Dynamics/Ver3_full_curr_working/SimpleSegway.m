function dX = SimpleSegway(t,X,p)

% Local variables
g = p.g;
mW = p.valM.mW;
mCM1 = p.valM.cm1;
mCM2 = p.valM.cm2;
mR = p.valM.mR;
L1 = p.valL.L1; 
L2 = p.valL.L2;
L3 = p.valL.L3;
R = p.R;
IW = p.IW; %
IK = p.IK;
IH = p.IH;
IR = p.IR;
theta1_lin = p.theta1_num;
theta2_lin = p.theta2_num;
theta3_lin = p.theta3_num;

L = [L1,L2,L3,R];
vMass = [mW,mCM1,mCM2,mR];

% States expanded 
xW = X(1); 
theta1 = X(2); 
theta2 = X(3);
theta3 = X(4);
dxW = X(5); 
dtheta1 = X(6); 
dtheta2 = X(7);
dtheta3 = X(8);

q = [xW,theta1,theta2,theta3];
dtheta = [dtheta1, dtheta2, dtheta3];
dq = [dxW,dtheta];

% Time indicator
t

% Set reference vectors
X_ref = [0;theta1_lin;theta2_lin;theta3_lin;0;0;0;0]; % Theta1 given. Found theta2 via COM calculations for stable point.
% tau_ref_10 = [0, -5.13599, -2.74328]';
tau_ref = fnc_tauLin();
tau_ref = tau_ref(2:4); 


% Mass and Coriolis+Gravity Matrices 
% M = [[                                                                                                                                       mCM1 + mCM2 + mR + mW + 3/(4000*R^2),                                                                                                                                                                                                                                                                                                                 mCM2*((9*L2*cos(theta1 + theta2))/10 + L1*cos(theta1)) + mR*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) + (1250*L1*mCM1*cos(theta1))/5763,                                                                                                                                                                                                                                                                                                                                                                                       mR*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)) + (9*L2*mCM2*cos(theta1 + theta2))/10,                                                                                                                                                                                                                L3*mR*cos(theta1 + theta2 + theta3)]
%     [mCM2*((9*L2*cos(theta1 + theta2))/10 + L1*cos(theta1)) + mR*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) + (1250*L1*mCM1*cos(theta1))/5763,                                                                                        mR*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3))^2 + mR*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3))^2 + mCM2*((9*L2*cos(theta1 + theta2))/10 + L1*cos(theta1))^2 + mCM2*((9*L2*sin(theta1 + theta2))/10 + L1*sin(theta1))^2 + (1562500*L1^2*mCM1*cos(theta1)^2)/33212169 + (1562500*L1^2*mCM1*sin(theta1)^2)/33212169 + 2683/12500, mR*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3))*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) + mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) + (9*L2*mCM2*sin(theta1 + theta2)*((9*L2*sin(theta1 + theta2))/10 + L1*sin(theta1)))/10 + (9*L2*mCM2*cos(theta1 + theta2)*((9*L2*cos(theta1 + theta2))/10 + L1*cos(theta1)))/10 + 10107/50000, L3*mR*cos(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) + L3*mR*sin(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) + 4933/25000]
%     [                                                                      mR*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)) + (9*L2*mCM2*cos(theta1 + theta2))/10, mR*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3))*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) + mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) + (9*L2*mCM2*sin(theta1 + theta2)*((9*L2*sin(theta1 + theta2))/10 + L1*sin(theta1)))/10 + (9*L2*mCM2*cos(theta1 + theta2)*((9*L2*cos(theta1 + theta2))/10 + L1*cos(theta1)))/10 + 10107/50000,                                                                                                                                                                                                                                                 mR*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3))^2 + mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))^2 + (81*L2^2*mCM2*cos(theta1 + theta2)^2)/100 + (81*L2^2*mCM2*sin(theta1 + theta2)^2)/100 + 10107/50000,                                   L3*mR*cos(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)) + L3*mR*sin(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)) + 4933/25000]
%     [                                                                                                                                        L3*mR*cos(theta1 + theta2 + theta3),                                                                                                                                                                                                                                          L3*mR*cos(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) + L3*mR*sin(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) + 4933/25000,                                                                                                                                                                                                                                                                            L3*mR*cos(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)) + L3*mR*sin(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)) + 4933/25000,                                                                                                                                                     mR*L3^2*cos(theta1 + theta2 + theta3)^2 + mR*L3^2*sin(theta1 + theta2 + theta3)^2 + 4933/25000]];
% 
% C =[                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      - dtheta3*(L3*dtheta1*mR*sin(theta1 + theta2 + theta3) + L3*dtheta2*mR*sin(theta1 + theta2 + theta3) + L3*dtheta3*mR*sin(theta1 + theta2 + theta3)) - dtheta1*(dtheta1*((mR*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)))/2 + (mCM2*((9*L2*sin(theta1 + theta2))/10 + L1*sin(theta1)))/2 + (625*L1*mCM1*sin(theta1))/5763) + (dtheta1*(mR*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) + mCM2*((9*L2*sin(theta1 + theta2))/10 + L1*sin(theta1)) + (1250*L1*mCM1*sin(theta1))/5763))/2 + (dtheta2*(mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)) + (9*L2*mCM2*sin(theta1 + theta2))/10))/2 + dtheta2*((mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)))/2 + (9*L2*mCM2*sin(theta1 + theta2))/20) + L3*dtheta3*mR*sin(theta1 + theta2 + theta3)) - dtheta2*((dtheta1*(mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)) + (9*L2*mCM2*sin(theta1 + theta2))/10))/2 + (dtheta2*(mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)) + (9*L2*mCM2*sin(theta1 + theta2))/10))/2 + dtheta1*((mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)))/2 + (9*L2*mCM2*sin(theta1 + theta2))/20) + dtheta2*((mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)))/2 + (9*L2*mCM2*sin(theta1 + theta2))/20) + L3*dtheta3*mR*sin(theta1 + theta2 + theta3))
% dxW*((dtheta1*(mR*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) + mCM2*((9*L2*sin(theta1 + theta2))/10 + L1*sin(theta1)) + (1250*L1*mCM1*sin(theta1))/5763))/2 + (dtheta2*(mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)) + (9*L2*mCM2*sin(theta1 + theta2))/10))/2 + (L3*dtheta3*mR*sin(theta1 + theta2 + theta3))/2) - dtheta3*((dtheta2*(L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) - L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) + L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)) - L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))))/2 + dtheta2*((L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)))/2 - (L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)))/2 + (L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)))/2 - (L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)))/2) + dtheta1*(L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) - L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3))) + (dtheta3*(L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) - L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3))))/2 + (dtheta1*(2*L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) - 2*L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3))))/2 + dtheta3*((L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)))/2 - (L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)))/2) + L3*dxW*mR*sin(theta1 + theta2 + theta3)) - dtheta2*((dtheta1*(2*mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) - 2*mR*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3))*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) + (9*L2*mCM2*sin(theta1 + theta2)*((9*L2*cos(theta1 + theta2))/10 + L1*cos(theta1)))/5 - (9*L2*mCM2*cos(theta1 + theta2)*((9*L2*sin(theta1 + theta2))/10 + L1*sin(theta1)))/5))/2 + dtheta1*(mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) - mR*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3))*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) + (9*L2*mCM2*sin(theta1 + theta2)*((9*L2*cos(theta1 + theta2))/10 + L1*cos(theta1)))/10 - (9*L2*mCM2*cos(theta1 + theta2)*((9*L2*sin(theta1 + theta2))/10 + L1*sin(theta1)))/10) + (dtheta2*(mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) - mR*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3))*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) + (9*L2*mCM2*sin(theta1 + theta2)*((9*L2*cos(theta1 + theta2))/10 + L1*cos(theta1)))/10 - (9*L2*mCM2*cos(theta1 + theta2)*((9*L2*sin(theta1 + theta2))/10 + L1*sin(theta1)))/10))/2 + dtheta2*((mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)))/2 - (mR*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3))*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)))/2 + (9*L2*mCM2*sin(theta1 + theta2)*((9*L2*cos(theta1 + theta2))/10 + L1*cos(theta1)))/20 - (9*L2*mCM2*cos(theta1 + theta2)*((9*L2*sin(theta1 + theta2))/10 + L1*sin(theta1)))/20) + (dxW*(mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)) + (9*L2*mCM2*sin(theta1 + theta2))/10))/2 + dxW*((mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)))/2 + (9*L2*mCM2*sin(theta1 + theta2))/20) + (dtheta3*(L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) - L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) - L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)) + L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))))/2 + dtheta3*((L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)))/2 - (L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)))/2 - (L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)))/2 + (L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)))/2)) - dtheta1*(dxW*((mR*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)))/2 + (mCM2*((9*L2*sin(theta1 + theta2))/10 + L1*sin(theta1)))/2 + (625*L1*mCM1*sin(theta1))/5763) + (dxW*(mR*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) + mCM2*((9*L2*sin(theta1 + theta2))/10 + L1*sin(theta1)) + (1250*L1*mCM1*sin(theta1))/5763))/2) - g*mR*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) - g*mCM2*((9*L2*sin(theta1 + theta2))/10 + L1*sin(theta1)) + (dtheta1*dxW*(mR*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) + mCM2*((9*L2*sin(theta1 + theta2))/10 + L1*sin(theta1)) + (1250*L1*mCM1*sin(theta1))/5763))/2 + (dtheta2*dxW*(mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)) + (9*L2*mCM2*sin(theta1 + theta2))/10))/2 - (1250*L1*g*mCM1*sin(theta1))/5763 + (L3*dtheta3*dxW*mR*sin(theta1 + theta2 + theta3))/2
%                dtheta3*((dtheta1*(L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) - L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) - L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)) + L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))))/2 + (L3*dxW*mR*sin(theta1 + theta2 + theta3))/2) - dtheta2*((dtheta1*(mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) - mR*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3))*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) + (9*L2*mCM2*sin(theta1 + theta2)*((9*L2*cos(theta1 + theta2))/10 + L1*cos(theta1)))/10 - (9*L2*mCM2*cos(theta1 + theta2)*((9*L2*sin(theta1 + theta2))/10 + L1*sin(theta1)))/10))/2 + dtheta1*((mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)))/2 - (mR*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3))*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)))/2 + (9*L2*mCM2*sin(theta1 + theta2)*((9*L2*cos(theta1 + theta2))/10 + L1*cos(theta1)))/20 - (9*L2*mCM2*cos(theta1 + theta2)*((9*L2*sin(theta1 + theta2))/10 + L1*sin(theta1)))/20) + (dxW*(mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)) + (9*L2*mCM2*sin(theta1 + theta2))/10))/2 + dxW*((mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)))/2 + (9*L2*mCM2*sin(theta1 + theta2))/20)) + dxW*((dtheta1*(mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)) + (9*L2*mCM2*sin(theta1 + theta2))/10))/2 + (dtheta2*(mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)) + (9*L2*mCM2*sin(theta1 + theta2))/10))/2 + (L3*dtheta3*mR*sin(theta1 + theta2 + theta3))/2) + dtheta1*((dtheta1*(2*mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) - 2*mR*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3))*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) + (9*L2*mCM2*sin(theta1 + theta2)*((9*L2*cos(theta1 + theta2))/10 + L1*cos(theta1)))/5 - (9*L2*mCM2*cos(theta1 + theta2)*((9*L2*sin(theta1 + theta2))/10 + L1*sin(theta1)))/5))/2 + (dtheta2*(mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) - mR*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3))*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) + (9*L2*mCM2*sin(theta1 + theta2)*((9*L2*cos(theta1 + theta2))/10 + L1*cos(theta1)))/10 - (9*L2*mCM2*cos(theta1 + theta2)*((9*L2*sin(theta1 + theta2))/10 + L1*sin(theta1)))/10))/2 + (dxW*(mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)) + (9*L2*mCM2*sin(theta1 + theta2))/10))/2 + (dtheta3*(L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) - L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) - L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)) + L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))))/2) + dtheta2*((dtheta1*(mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) - mR*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3))*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) + (9*L2*mCM2*sin(theta1 + theta2)*((9*L2*cos(theta1 + theta2))/10 + L1*cos(theta1)))/10 - (9*L2*mCM2*cos(theta1 + theta2)*((9*L2*sin(theta1 + theta2))/10 + L1*sin(theta1)))/10))/2 + (dxW*(mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)) + (9*L2*mCM2*sin(theta1 + theta2))/10))/2) - dtheta1*((dxW*(mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)) + (9*L2*mCM2*sin(theta1 + theta2))/10))/2 + dxW*((mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)))/2 + (9*L2*mCM2*sin(theta1 + theta2))/20)) - dtheta3*((dtheta1*(L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) - L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) + L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)) - L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))))/2 + dtheta1*((L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)))/2 - (L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)))/2 + (L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)))/2 - (L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)))/2) + dtheta2*(L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)) - L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))) + (dtheta3*(L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)) - L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))))/2 + (dtheta2*(2*L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)) - 2*L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))))/2 + dtheta3*((L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)))/2 - (L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)))/2) + L3*dxW*mR*sin(theta1 + theta2 + theta3)) - g*mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)) - (9*L2*g*mCM2*sin(theta1 + theta2))/10
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        dtheta1*((dtheta2*(L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) - L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) + L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)) - L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))))/2 + (dtheta3*(L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) - L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3))))/2 + (dtheta1*(2*L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) - 2*L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3))))/2 + (L3*dxW*mR*sin(theta1 + theta2 + theta3))/2) - dtheta3*((dtheta2*(L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)) - L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))))/2 + dtheta2*((L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)))/2 - (L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)))/2) + (dtheta1*(L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) - L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3))))/2 + dtheta1*((L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)))/2 - (L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)))/2) + L3*dxW*mR*sin(theta1 + theta2 + theta3)) + dxW*((L3*dtheta1*mR*sin(theta1 + theta2 + theta3))/2 + (L3*dtheta2*mR*sin(theta1 + theta2 + theta3))/2 + (L3*dtheta3*mR*sin(theta1 + theta2 + theta3))/2) + dtheta3*((dtheta2*(L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)) - L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))))/2 + (dtheta1*(L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) - L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3))))/2 + (L3*dxW*mR*sin(theta1 + theta2 + theta3))/2) - dtheta2*((dtheta1*(L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) - L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) - L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)) + L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))))/2 + dtheta1*((L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)))/2 - (L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)))/2 - (L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)))/2 + (L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)))/2) + L3*dxW*mR*sin(theta1 + theta2 + theta3)) + dtheta2*((dtheta1*(L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) - L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) + L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)) - L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))))/2 + (dtheta3*(L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)) - L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))))/2 + (dtheta2*(2*L3*mR*sin(theta1 + theta2 + theta3)*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)) - 2*L3*mR*cos(theta1 + theta2 + theta3)*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3))))/2 + (L3*dxW*mR*sin(theta1 + theta2 + theta3))/2) - L3*g*mR*sin(theta1 + theta2 + theta3) - L3*dtheta1*dxW*mR*sin(theta1 + theta2 + theta3)];

M = fnc_H(q,L,vMass);
C = fnc_C(q,dq,L,vMass,g);

% Calculating conditioning of matrix
% [eigVec, eigVal] = eig(M);
% eigVal_ar = diag(eigVal);
% cond = max(eigVal_ar)/min(eigVal_ar);

%Calculate initial torques
K = fnc_K();
X = X - X_ref;
tau = -K*X;

% Torque-speed saturation (linear)
% for i = 1:3
%         %Adding torque saturation
%      if p.enableSaturation == "linear"
%         % max_stall_torque = 2.83 | no_load_speed = 209.4 rad/s | Gear ratio = 6
%         tau_bg_max = 2.8 - (2.8/209)*abs(dtheta(i)); %Fitting the curve 
%         if(tau_bg_max < 0)
%             tau_bg_max = 0;
%         end
%         tau_bg_min = -tau_bg_max;
%         tau_g_max = 6*tau_bg_max;
%         tau_g_min = 6*tau_bg_min;
%         if tau(i) > tau_g_max
%             tau(i) = tau_g_max;
%         elseif tau(i) < tau_g_min
%             tau(i) = tau_g_min;
%         end
%         
%     elseif p.enableSaturation == "cutoff"
%         if tau(i) > 17
%             tau(i) = 17;  
%         elseif tau(i) < -17
%             tau(i) = -17;
%         end
%      end
% end
%

% Ensure we normalize properly around linearization 
tau = tau + tau_ref;

% Solving for accelerations and adding damping
tau = [tau(1)/R; -tau(1); -tau(2); -tau(3)];

u = 0.5*[dxW; dtheta1; dtheta2; dtheta3] + tau;
ddq = M\(u - C);

%Testing quadprog2
% Qq = diag([1000 1 10 1 10 10 100 100]);
% Ru = diag([1 1 1]);
% H = [Qq zeros(8,4);zeros(3,8) Ru zeros(3,1); zeros(1,11) 1];
% f = 0;
% Aeq = [A B 0; zeros(1,11) 1];
% Fext_z = - dtheta2*(dtheta1*(mR*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)) + (L2*mCM2*cos(theta1 + theta2))/2) + dtheta2*(mR*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)) + (L2*mCM2*cos(theta1 + theta2))/2) + L3*dtheta3*mR*cos(theta1 + theta2 + theta3)) - g*mR - ddtheta1*(mR*(L2*sin(theta1 + theta2) + L1*sin(theta1) + L3*sin(theta1 + theta2 + theta3)) + mCM2*((L2*sin(theta1 + theta2))/2 + L1*sin(theta1)) + (L1*mCM1*sin(theta1))/2) - ddtheta2*(mR*(L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3)) + (L2*mCM2*sin(theta1 + theta2))/2) - dtheta1*(dtheta2*(mR*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)) + (L2*mCM2*cos(theta1 + theta2))/2) + dtheta1*(mCM2*((L2*cos(theta1 + theta2))/2 + L1*cos(theta1)) + mR*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) + (L1*mCM1*cos(theta1))/2) + L3*dtheta3*mR*cos(theta1 + theta2 + theta3)) - L3*ddtheta3*mR*sin(theta1 + theta2 + theta3) - L3*dtheta3*mR*cos(theta1 + theta2 + theta3)*(dtheta1 + dtheta2 + dtheta3);
% beq = [[dxW; dtheta1; dtheta2; dtheta3]; [ddq(1); ddq(2); ddq(3); ddq(4)]; Fext_z];


dX = [[dxW; dtheta1; dtheta2; dtheta3]; [ddq(1); ddq(2); ddq(3); ddq(4)]];

end