%% FILE PARAMS
close all;
addpath functions;
p = getParams();
p.enableSaturation = "cutoff"; %{cutoff,linear,none}
p.captureVideoEnable = false;
simTime = 4.0;

%% STATE SPACE
A = fnc_A();
B = fnc_B();
A = [[zeros(4,4) eye(4)]; A]; 
B = [zeros(4,3); B];   
% [A,B] = stateSpace('10'); %Send number (char) of angle we want to linearize around i.e 10 degrees = pi/18

%% LQR CONTROLLER
%w = (p.g/p.valL.L1)^0.5;
%Qq = diag([1 1/R 1/w 1/(w*Rx)]);

%Witout initial vel
Qq = diag([1000 1 10 10 1000 1 10 10]); %This worked (not work w 4 m/s ic)
% Qpos = diag([.1 1 2 2]);
% Qvel = diag([10 10 10 10]);
% Qq = blkdiag(Qpos, Qvel);
Ru = diag([1 10 3]); 
% Ru = diag([100 2 10]); 


%With inital vel
% Qq = diag([10 1000 1000 1 150 10 10 10]); %This did not. dtheta3 cost < 100
% Ru = diag([2 10 10]); 

K = lqr(A,B,Qq,Ru)
write_fcn_m('fnc_K.m',{},[],{K,'K'});
vpa(eig((A-B*K)))

%% SIMULATION
q0 = [0; p.theta1_num; p.theta2_num; p.theta3_num; 0; 0; 0; 0];
[T,X] = ode45(@(t,X)SimpleSegway(t,X,p),[0 simTime],q0);

% Output States
xW = X(:,1); 
theta1 = X(:,2);
theta2 = X(:,3);
theta3 = X(:,4);
dxW = X(:,5); 
dtheta1 = X(:,6); 
dtheta2 = X(:,7);
dtheta3 = X(:,8);

%% TORQUE CALCULATION
[tao,tao_desired] = calculateTorques(X,p);
%%
testing_X = [0,0,0,0,0,0,0,0];
testing_tao = zeros(1,3);
GRFz = calculateGRF(testing_X,p,testing_tao);
% GRFz = calculateGRF(X,p,tao);

% 
%% ANIMATION
%animatingRobot(T,X,p,tao);
animatingOnlyRobot(T,X,p);
%animate3DRobot(T,X);

%% ANIMATING SINGLE FRAME
L = [p.valL.L1,p.valL.L2,p.valL.L3];
M = [p.valM.cm1,p.valM.cm2,p.valM.mR];
xW = 0;
% theta1 = deg2rad(0);
% [theta2,theta3] = solveJointAngles(theta1,1,M,L,1);
% theta = [theta1,theta2,theta3];
theta1 = 0;
theta2 = pi/2;
theta3 = -pi/4;
pitch = pi/36;
% [theta1c,theta2c,theta3c] = motorToControlAngles([theta1,theta2,theta3],pitch);
% theta = [theta1c,theta2c,theta3c];
theta = [p.theta1_num,p.theta2_num,p.theta3_num]
q = [xW, theta];
fSingle = figure(100);
ax=axes('Parent',fSingle);
SATYRR_Visualize(q,L,ax);

%% GRAPHING 
limx = 2;

desiredState = ones(length(T), 4);
desiredState(:,1) = 0 * desiredState(:,1);
desiredState(:,2) = angle_of_linearization * desiredState(:,2);
desiredState(:,3) = -angle_of_linearization * desiredState(:,3);
desiredState(:,4) = 0 * desiredState(:,4);


figure(1);
subplot(1,2,1)
plot(T,X(:,1:4));
%hold on;
%plot(T,desiredState);
title('Positions')
xlim([0 limx]);
ylim([-5 5]);
xlabel('Time (sec)');
ylabel('Meters or Radians');
legend('xW', '\theta_{1}', '\theta_{2}', '\theta_{3}');
%%
subplot(1,2,2)
plot(T,X(:,5:8));
title('Velocity')
xlim([0 limx]);
ylim([-5 5]);
xlabel('Time (sec)');
ylabel('Rads/sec');
name = legend('$\dot{xW}$', '$\dot{\theta_{1}}$', '$\dot{\theta_{2}}$', '$\dot{\theta_{3}}$');
set(name,'Interpreter','latex');

figure(2);
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

figure(3)
plot(T,GRFz);
title('GRFz')
xlim([0 limx]);
xlabel('Time (sec)');
ylabel('Force (N)');
% ylim([-10,200]);