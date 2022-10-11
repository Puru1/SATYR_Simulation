<<<<<<< HEAD
<<<<<<< HEAD
%% FILE PARAMS
clc;
clearvars -except f q dq ddq;
close all;
addpath functions;

p = getParams();
%% FILE FEATURE SETTINGS
p.enableSaturation = "cutoff"; %{cutoff,linear,none}
p.captureVideoEnable = false;
% p.Ks = ((p.valM.mB + p.valM.mW)*p.g)/(2*p.deltaL);
simTime = 5.0;

%% LOCAL VARIBALES

%% STATE SPACE
A = fnc_A();
B = fnc_B();
A = [[zeros(3,3) eye(3)]; A]; 
B = [zeros(3,2); B];   
% [A,B] = stateSpace('10'); %Send number (char) of angle we want to linearize around i.e 10 degrees = pi/18

%% LQR CONTROLLER
%w = (p.g/p.valL.L1)^0.5;
%Qq = diag([1 1/R 1/w 1/(w*Rx)]);

%Witout initial vel
% Qq = diag([1000 1 10 10 1000 1 10 10]); %This worked (not work w 4 m/s ic)
% Qpos = diag([.1 50 500]);
% Qvel = diag([1 .01 50]);
Qpos = diag([.01 50 1]);
Qvel = diag([1 1 10]);
Qq = blkdiag(Qpos, Qvel);
Ru = diag([10 2]); 
% Ru = diag([100 2 10]); 


%With inital vel
% Qq = diag([10 1000 1000 1 150 10 10 10]); %This did not. dtheta3 cost < 100
% Ru = diag([2 10 10]); 

K = lqr(A,B,Qq,Ru)
% K(:,1) = 0;
% K(1,6) = -.37;
write_fcn_m('fnc_K.m',{},[],{K,'K'});
vpa(eig((A-B*K)))
%% SIMULATION
q0 = [0; .365; pi/45; 0; 0; 0];
[T,X] = ode45(@(t,X)SimpleSegway(t,X,p),[0 simTime],q0);

% Output States
xW = X(:,1);
l_h = X(:,2);
theta1 = X(:,3);
dxW = X(:,4); 
dl_h = X(:,5); 
dtheta1 = X(:,6);

%% TORQUE CALCULATION
% [tao,tao_desired] = calculateTorques(X,p);
%%
% testing_X = [0,0,0,0,0,0,0,0];
% testing_tao = zeros(1,3);
% GRFz = calculateGRF(testing_X,p,testing_tao);
% GRFz = calculateGRF(X,p,tao);

% 
%% ANIMATION
animatingWSLIP(T,X,p);
%animate3DRobot(T,X);

%% ANIMATING SINGLE FRAME
THETA1_R = pi/18;
xW = 0;
LH = .76;
L = p.L;
[theta1_f, theta2_f] = solveJointAngles(THETA1_R,LH,p);
q_vis = [xW, THETA1_R + vpa(theta1_f), vpa(-theta1_f+theta2_f),-vpa(theta2_f)];
fSingle = figure(99);
ax=axes('Parent',fSingle);
SATYRR_Visualize(q_vis,L,ax);

%% GRAPHING 
limx = .2;

desiredState = ones(length(T), 4);
desiredState(:,1) = 0 * desiredState(:,1);
desiredState(:,2) = p.valL.L1 + p.valL.L2 + p.valL.L3;
desiredState(:,3) = 0;


figure(1);
% subplot(1,2,1)
plot(T,X(:,1:3));
%hold on;
%plot(T,desiredState);
title('Positions')
xlim([0 limx]);
ylim([-5 5]);
xlabel('Time (sec)');
ylabel('Meters or Radians');
legend('xW', 'l_{h}', '\theta_{1}');

figure(2);
plot(T,X(:,4:6));
title('Velocity')
xlim([0 limx]);
ylim([-5 5]);
xlabel('Time (sec)');
ylabel('Rads/sec');
name = legend('$\dot{xW}$', '$\dot{l_{h}}$', '$\dot{\theta_{1}}$');
set(name,'Interpreter','latex');
%%
figure(3);
subplot(1,2,1)
plot(T, tao);
title('Realized Torques')
xlim([0 limx]);
xlabel('Time (sec)');
ylabel('Torque (N*m)');
legend("tao_{1}", "tao_{2}", "tao_{3}"); 

%This desired doesnt really make sense since the previous and future
%state are effected by the applied control input
subplot(1,2,2)
plot(T, tao_desired);
title('Desired Torques')
xlim([0 limx]);
xlabel('Time (sec)');
ylabel('Torque (N*m)');
legend("tao_{1}", "tao_{2}", "tao_{3}"); 

figure(4)
plot(T,GRFz);
title('GRFz')
xlim([0 limx]);
xlabel('Time (sec)');
ylabel('Force (N)');
=======
%% FILE PARAMS
clc;
clearvars -except f q dq ddq;
close all;
addpath functions;

p = getParams();
%% FILE FEATURE SETTINGS
p.enableSaturation = "cutoff"; %{cutoff,linear,none}
p.captureVideoEnable = false;
simTime = 5.0;

%% LOCAL VARIBALES

%% STATE SPACE
A = fnc_A();
B = fnc_B();
A = [[zeros(3,3) eye(3)]; A]; 
B = [zeros(3,2); B];   
% [A,B] = stateSpace('10'); %Send number (char) of angle we want to linearize around i.e 10 degrees = pi/18

%% LQR CONTROLLER
%w = (p.g/p.valL.L1)^0.5;
%Qq = diag([1 1/R 1/w 1/(w*Rx)]);

%Witout initial vel
% Qq = diag([1000 1 10 10 1000 1 10 10]); %This worked (not work w 4 m/s ic)
Qpos = diag([5 50 100]);
Qvel = diag([200 10 50]);
Qq = blkdiag(Qpos, Qvel);
Ru = diag([1 5]); 
% Ru = diag([100 2 10]); 


%With inital vel
% Qq = diag([10 1000 1000 1 150 10 10 10]); %This did not. dtheta3 cost < 100
% Ru = diag([2 10 10]); 

K = lqr(A,B,Qq,Ru);
write_fcn_m('fnc_K.m',{},[],{K,'K'});
vpa(eig((A-B*K)))
%% SIMULATION
q0 = [0; .60; pi/18; 0; 0; 0];
[T,X] = ode45(@(t,X)SimpleSegway(t,X,p),[0 simTime],q0);

% Output States
xW = X(:,1);
l_h = X(:,2);
theta1 = X(:,3);
dxW = X(:,4); 
dl_h = X(:,5); 
dtheta1 = X(:,6);

%% TORQUE CALCULATION
% [tao,tao_desired] = calculateTorques(X,p);
%%
% testing_X = [0,0,0,0,0,0,0,0];
% testing_tao = zeros(1,3);
% GRFz = calculateGRF(testing_X,p,testing_tao);
% GRFz = calculateGRF(X,p,tao);

% 
%% ANIMATION
animatingWSLIP(T,X,p);
%animate3DRobot(T,X);

%% ANIMATING SINGLE FRAME
THETA1_R = pi/18;
xW = 0;
LH = .76;
L = p.L;
[theta1_f, theta2_f] = solveJointAngles(THETA1_R,LH,p);
q_vis = [xW, THETA1_R + vpa(theta1_f), vpa(-theta1_f+theta2_f),-vpa(theta2_f)];
fSingle = figure(99);
ax=axes('Parent',fSingle);
SATYRR_Visualize(q_vis,L,ax);

%% GRAPHING 
limx = 1.25;

desiredState = ones(length(T), 4);
desiredState(:,1) = 0 * desiredState(:,1);
desiredState(:,2) = p.valL.L1 + p.valL.L2 + p.valL.L3;
desiredState(:,3) = 0;


figure(1);
% subplot(1,2,1)
plot(T,X(:,1:3));
%hold on;
%plot(T,desiredState);
title('Positions')
xlim([0 limx]);
ylim([-5 5]);
xlabel('Time (sec)');
ylabel('Meters or Radians');
legend('xW', 'l_{h}', '\theta_{1}');

figure(2);
plot(T,X(:,4:6));
title('Velocity')
xlim([0 limx]);
ylim([-5 5]);
xlabel('Time (sec)');
ylabel('Rads/sec');
name = legend('$\dot{xW}$', '$\dot{l_{h}}$', '$\dot{\theta_{1}}$');
set(name,'Interpreter','latex');
%%
figure(3);
subplot(1,2,1)
plot(T, tao);
title('Realized Torques')
xlim([0 limx]);
xlabel('Time (sec)');
ylabel('Torque (N*m)');
legend("tao_{1}", "tao_{2}", "tao_{3}"); 

%This desired doesnt really make sense since the previous and future
%state are effected by the applied control input
subplot(1,2,2)
plot(T, tao_desired);
title('Desired Torques')
xlim([0 limx]);
xlabel('Time (sec)');
ylabel('Torque (N*m)');
legend("tao_{1}", "tao_{2}", "tao_{3}"); 

figure(4)
plot(T,GRFz);
title('GRFz')
xlim([0 limx]);
xlabel('Time (sec)');
ylabel('Force (N)');
>>>>>>> b6f1ecc8a3fb2eda4e78b6658502c98c386c0207
=======
%% FILE PARAMS
clc;
clearvars -except f q dq ddq;
close all;
addpath functions;

p = getParams();
%% FILE FEATURE SETTINGS
p.enableSaturation = "cutoff"; %{cutoff,linear,none}
p.captureVideoEnable = false;
% p.Ks = ((p.valM.mB + p.valM.mW)*p.g)/(2*p.deltaL);
simTime = 5.0;

%% LOCAL VARIBALES

%% STATE SPACE
A = fnc_A();
B = fnc_B();
A = [[zeros(3,3) eye(3)]; A]; 
B = [zeros(3,2); B];   
% [A,B] = stateSpace('10'); %Send number (char) of angle we want to linearize around i.e 10 degrees = pi/18

%% LQR CONTROLLER
%w = (p.g/p.valL.L1)^0.5;
%Qq = diag([1 1/R 1/w 1/(w*Rx)]);

%Witout initial vel
% Qq = diag([1000 1 10 10 1000 1 10 10]); %This worked (not work w 4 m/s ic)
% Qpos = diag([.1 50 500]);
% Qvel = diag([1 .01 50]);
Qpos = diag([.01 50 1]);
Qvel = diag([1 1 10]);
Qq = blkdiag(Qpos, Qvel);
Ru = diag([10 2]); 
% Ru = diag([100 2 10]); 


%With inital vel
% Qq = diag([10 1000 1000 1 150 10 10 10]); %This did not. dtheta3 cost < 100
% Ru = diag([2 10 10]); 

K = lqr(A,B,Qq,Ru)
% K(:,1) = 0;
% K(1,6) = -.37;
write_fcn_m('fnc_K.m',{},[],{K,'K'});
vpa(eig((A-B*K)))
%% SIMULATION
q0 = [0; .365; pi/45; 0; 0; 0];
[T,X] = ode45(@(t,X)SimpleSegway(t,X,p),[0 simTime],q0);

% Output States
xW = X(:,1);
l_h = X(:,2);
theta1 = X(:,3);
dxW = X(:,4); 
dl_h = X(:,5); 
dtheta1 = X(:,6);

%% TORQUE CALCULATION
% [tao,tao_desired] = calculateTorques(X,p);
%%
% testing_X = [0,0,0,0,0,0,0,0];
% testing_tao = zeros(1,3);
% GRFz = calculateGRF(testing_X,p,testing_tao);
% GRFz = calculateGRF(X,p,tao);

% 
%% ANIMATION
animatingWSLIP(T,X,p);
%animate3DRobot(T,X);

%% ANIMATING SINGLE FRAME
THETA1_R = pi/18;
xW = 0;
LH = .76;
L = p.L;
[theta1_f, theta2_f] = solveJointAngles(THETA1_R,LH,p);
q_vis = [xW, THETA1_R + vpa(theta1_f), vpa(-theta1_f+theta2_f),-vpa(theta2_f)];
fSingle = figure(99);
ax=axes('Parent',fSingle);
SATYRR_Visualize(q_vis,L,ax);

%% GRAPHING 
limx = .2;

desiredState = ones(length(T), 4);
desiredState(:,1) = 0 * desiredState(:,1);
desiredState(:,2) = p.valL.L1 + p.valL.L2 + p.valL.L3;
desiredState(:,3) = 0;


figure(1);
% subplot(1,2,1)
plot(T,X(:,1:3));
%hold on;
%plot(T,desiredState);
title('Positions')
xlim([0 limx]);
ylim([-5 5]);
xlabel('Time (sec)');
ylabel('Meters or Radians');
legend('xW', 'l_{h}', '\theta_{1}');

figure(2);
plot(T,X(:,4:6));
title('Velocity')
xlim([0 limx]);
ylim([-5 5]);
xlabel('Time (sec)');
ylabel('Rads/sec');
name = legend('$\dot{xW}$', '$\dot{l_{h}}$', '$\dot{\theta_{1}}$');
set(name,'Interpreter','latex');
%%
figure(3);
subplot(1,2,1)
plot(T, tao);
title('Realized Torques')
xlim([0 limx]);
xlabel('Time (sec)');
ylabel('Torque (N*m)');
legend("tao_{1}", "tao_{2}", "tao_{3}"); 

%This desired doesnt really make sense since the previous and future
%state are effected by the applied control input
subplot(1,2,2)
plot(T, tao_desired);
title('Desired Torques')
xlim([0 limx]);
xlabel('Time (sec)');
ylabel('Torque (N*m)');
legend("tao_{1}", "tao_{2}", "tao_{3}"); 

figure(4)
plot(T,GRFz);
title('GRFz')
xlim([0 limx]);
xlabel('Time (sec)');
ylabel('Force (N)');
>>>>>>> adding_folders
% ylim([-10,200]);