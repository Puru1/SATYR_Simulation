function dX = sim_cartPole(t,X,p,X_ref)

% States expanded 
xW = X(1); 
theta1 = X(2); 
dxW = X(3); 
dtheta1 = X(4); 

q = [xW;theta1;dxW;dtheta1];

% DCM conversion
p.L1 = (.35329 - .141)*abs(sin(t)) + .141;

h_R = .35329;
omega_R = 1/sqrt(p.g/h_R);
xCoM = xW + h_R*theta1; 
DCM = theta1 + dtheta1/omega_R;
CCM = theta1 - dtheta1/omega_R;

q_DCM = p.T * q;

% Time indicator
t

% Set reference vectors
tau_ref = fnc_tauLin();
%K = fnc_K();

K_low = [135,-70,0,-343];
K_high = [-276,-66,0,-264];
K = K_low + ((K_high - K_low)/(.35329 - .141))*(p.L1- .141);
%K_orig = fnc_K_orig();

% Mass and Coriolis+Gravity Matrices 
M = fnc_H(q,p);
C = fnc_C(q,p);

%Calculate initial torques
X = (q_DCM - X_ref); % DCM X error 
tau_error = -K*X;

% Ensure we normalize properly around linearization 
tau = tau_error + tau_ref(2);

% Solving for accelerations and adding damping
u = [tau; 0]; % double check why the input is a 2x1 instead of a 1x1
ddq = M\(u - C);

dX = [[dxW; dtheta1]; [ddq(1); ddq(2)]];

end