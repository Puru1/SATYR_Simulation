function dX = sim_invPendulum(t,X,p,X_ref)

global Ki X_prev t_prev;

% States expanded 
xW = X(1); 
theta1 = X(2); 
dxW = X(3); 
dtheta1 = X(4); 

q = [xW;theta1];
dq = [dxW;dtheta1];

% Time indicator
t

%%
% X_ref = sampleJoystick_traj(p,t);

% Set reference vectors
% X_ref = [0;p.theta1_num;0;0]; % Theta1 given. Found theta2 via COM calculations for stable point.
% X_ref = [0;0;.5;0]; % Theta1 given. Found theta2 via COM calculations for stable point.
% tau_ref = fnc_C([X_ref(1),X_ref(2)],[X_ref(3),X_ref(4)],[p.mW,p.mCM],p.L1);
tau_ref = fnc_tauLin();
K = fnc_K();
%tau_lin = - vpa(subs(C,[q' dq' g mCM L1],[states_lin p.g p.mCM p.L1]));

% Mass and Coriolis+Gravity Matrices 
M = fnc_H(q,[p.mW,p.mCM],p.L1,p.R);
C = fnc_C(q,dq,[p.mW,p.mCM],p.L1);

% Calculating conditioning of matrix
% [eigVec, eigVal] = eig(M);
% eigVal_ar = diag(eigVal);
% cond = max(eigVal_ar)/min(eigVal_ar);

%Calculate initial torques
X = X - X_ref; % X error 
tau_error = -K*X;

%Testing Integral Control
% X_int = X_prev + (X*(t - t_prev));
% tau_integral = -Ki*X_int;
% tau_error = tau_error + tau_integral;
% t_prev = t;
% X_prev = X;

% Ensure we normalize properly around linearization 
tau_des = tau_error + tau_ref(2);

% Actuator torque bound check
% [tau_max, tau_min, omega_out, actuatorFlag] = checkTorque(dq,tau_des);
% dtheta1 = omega_out;

% GRFz > 0 check
% grfFlag = checkGRF(p,q,dq,tau_des);

%  if (grfFlag == true) && (actuatorFlag)
%      tau = tau_des;
%  else
%     % Quadprog
%     fprintf('Quadprog required');
%     JvCM = fnc_JvCM(q,p.L1);%Wheel to COM Jacobian
%     JvCM_reduced = [JvCM(1,:);JvCM(3,:)]; %get rid of all zeros row
%     JvCM_inv = inv(-JvCM_reduced');
% 
%     H = 2*(p.Ru);
%     f = -tau_ref(2)' * (p.Ru);
%     A = JvCM_inv(:,2);
%     b = [0;0];
%     Aeq = [];
%     beq = [];
%     lb = tau_min
%     ub = tau_max
%     %x0 = tau_des;
%     %options = optimset('Display', 'off');
%     %opts = optimoptions(@quadprog,'Algorithm','active-set');
%     tau_error = quadprog(H,f,A,b,Aeq,beq,lb,ub); % WHY IS THIS TAU_ERROR
%     tau = tau_error + tau_ref(2)
%  end 

% Solving for accelerations and adding damping
% tau = [tau/(p.R); -tau];
tau = [tau_des/(p.R); -tau_des];
% u = 0.5*[dxW; dtheta1] + tau;
u = tau;
ddq = M\(u - C);

dX = [[dxW; dtheta1]; [ddq(1); ddq(2)]];


end