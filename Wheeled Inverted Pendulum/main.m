%% FILE PARAMS
clc;
clear all;
close all;
addpath fnc;

global enableSaturation  ;% Global feature settings
global captureVideoEnable; 
global K; % Global controller params

%% FILE FEATURE SETTINGS
enableSaturation = "linear"; %{cutoff,linear,none}
captureVideoEnable = false;
simTime = 4.0;

%% LOCAL VARIBALES
p = getParams();

%% STATE SPACE
A = fnc_A();
B = fnc_B();
A = [[zeros(2,2) eye(2)]; A]; 
B = [zeros(2,1); B];   
C = eye(4);
D = 0;
dt = .0015;

% Discretized system 
sys = ss(A,B,C,D);
d_sys = c2d(sys,dt);
A_dis = d_sys.A
B_dis = d_sys.B

% A_dis = [[ 1.0000  -0.0000    0.0015    -0.0000
%            0        1.0001         0     0.0015
%            0       -0.0447    1.0000    -0.0000
%            0        0.1377         0     1.0001]];
%        
% B_dis =  [0.0000 -0.0000 0.0147 -0.0355]';

% System ID'd Matrices 
A_sysId = [[ 1.0008   -0.0008    0.0515   -0.0424];
           [ 0.0040    0.9951    0.3213   -0.2412];
           [ 0.0097   -0.0000    0.9677   -0.0102];
           [-0.0000    0.0091    0.0137    0.9190]];
       
B_sysId = [0 0 .0005 -.0016]';
%% LQR CONTROLLER
%w = (p.g/p.valL.L1)^0.5;
%Qq = diag([1 1/R 1/w 1/(w*Rx)]);

%Witout initial vel
% Qq = diag([1000 1 10 10 1000 1 10 10]); %This worked (not work w 4 m/s ic)
% Ru = diag([1 10 3]); 

%With inital vel
% Qq = diag([(6*10^3) (5.5*10^4) 11 1]); % [xW theta dxW dtheta]
Qq = diag([(3.25*10^4) (2.65*10^5) (1*10^2) (2*10^2)]); % [xW theta dxW dtheta]
Ru = 1;
p.Qq = Qq;
p.Ru = Ru;

% Continuous time controller
K = lqr(A,B,Qq,Ru)
write_fcn_m('fnc_K.m',{},[],{K,'K'});
eig((A-B*K))

% Discrete time Controller
K_dis = dlqr(A_dis,B_dis, Qq, Ru);
K_dis = [-70 -335 -40 -33];
eig((A_dis-B_dis*K_dis));

% Identified System Controller
% K_model = dlqr(A_model,B_model, Qq, Ru)
%eig((A_model-B_model*K_model))

% K(:,1) = 01

%% SIMULATION
global Ki X_prev t_prev;
q0 = [0; 0; 0; 0];
X_ref = [0;.1;0;0];
X_prev = q0;
Ki = [-100 -1 -1 -1];
t_prev = 0;
%options = odeset('Events',@(t,X)robot_stopped(t,X,p));

[T,X] = ode45(@(t,X)sim_invPendulum(t,X,p,X_ref),[0 simTime],q0);
out = computeTorques(X,p,X_ref); % out(1) -> Fx |  out(2) -> tau

%% State Reconstruction
X(:,1) = X(:,1) + X_ref(1); 
X(:,2) = X(:,2) + X_ref(2);
X(:,3) = X(:,3) + X_ref(3); 
X(:,4) = X(:,4) + X_ref(4); 

xW = X(:,1);
theta1 = X(:,2);
dxW = X(:,3);
dtheta1 = X(:,4);

%% ANIMATION
animatePendulum(T,X,p);

%% GRAPHING 
limx = simTime;

figure(1);
subplot(1,2,1)
plot(T,X(:,1:2));
%hold on;
%plot(T,desiredState);
title('Positions (Des Vel = .5 m/s)')
xlim([0 limx]);
xlabel('Time (sec)');
ylabel('Meters or Radians');
legend('xW', '\theta_{1}');

subplot(1,2,2)
plot(T,X(:,3:4));
title('Velocity')
xlim([0 limx]);
xlabel('Time (sec)');
ylabel('Rads/sec');
name = legend('$\dot{xW}$', '$\dot{\theta_{1}}$');
set(name,'Interpreter','latex');

figure(2)
endIndex = 200;
plot(T(1:endIndex),out(1:endIndex,2));
title('Applied Wheel Torque');
xlabel('Time (sec)');
ylabel('Nm');
ylim([-20 20]);


