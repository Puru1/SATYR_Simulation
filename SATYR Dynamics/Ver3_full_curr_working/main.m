%% FILE PARAMS
clc;
clearvars -except f q dq ddq;
close all;
addpath functions;

global enableSaturation;% Global feature settings
global K; % Global controller params

%% FILE FEATURE SETTINGS
enableSaturation = "cutoff"; %{cutoff,linear,none}
captureVideoEnable = true;
simTime = 4.0;

%% LOCAL VARIBALES
p = getParams();
angle_of_linearization = p.theta2_num; %Used in graphing

%% STATE SPACE
[A,B] = stateSpace('10'); %Send number (char) of angle we want to linearize around i.e 10 degrees = pi/18

%% LQR CONTROLLER
%w = (p.g/p.valL.L1)^0.5;
%Qq = diag([1 1/R 1/w 1/(w*Rx)]);

%Witout initial vel
Qq = diag([1000 1 10 10 1000 1 10 10]); %This worked (not work w 4 m/s ic)
Ru = diag([1 10 3]); 

%With inital vel
% Qq = diag([10 1000 1000 1 150 10 10 10]); %This did not. dtheta3 cost < 100
% Ru = diag([2 10 10]); 

K = lqr(A,B,Qq,Ru);
K(:,1) = 0;

%% SIMULATION
q0 = [0; pi/8; -pi/10; 0; 0; 0; 0; 0];
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
% 
%% ANIMATION
animatingRobot(T,X,p);
%animate3DRobot(T,X);

%% ANIMATING SINGLE FRAME
% singleFrameFigure(p);
theta = [0,pi/6,0];
xW = 0;
q = [xW, theta];
L = [1, 1, 1];
CoM = comCalc(q,L);
% com = [test, l2Com, l3Com];
SATYRR_Visualize(theta,L,CoM);

%% GRAPHING 
limx = simTime;

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
xlabel('Time (sec)');
ylabel('Meters or Radians');
legend('xW', '\theta_{1}', '\theta_{2}', '\theta_{3}');

subplot(1,2,2)
plot(T,X(:,5:8));
title('Velocity')
xlim([0 limx]);
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

