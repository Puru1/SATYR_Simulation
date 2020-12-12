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
captureVideoEnable = true;
simTime = 4;

%% LOCAL VARIBALES
p = getParams();

%% STATE SPACE
A = [[ 0, -13.756078252842860846181310404154, 0, 0]
    [ 0,  101.86106052724123642956666728109, 0, 0]];

B =[ 3.5972055296288989539967736039851
    -17.518911774690730080772003219694];

A = [[zeros(2,2) eye(2)]; A]; 
B = [zeros(2,1); B];         
%% LQR CONTROLLER
%w = (p.g/p.valL.L1)^0.5;
%Qq = diag([1 1/R 1/w 1/(w*Rx)]);

%Witout initial vel
% Qq = diag([1000 1 10 10 1000 1 10 10]); %This worked (not work w 4 m/s ic)
% Ru = diag([1 10 3]); 

%With inital vel
Qq = diag([1 1 1 1]); %This did not. dtheta3 cost < 100
Ru = 1;
p.Qq = Qq;
p.Ru = Ru;

K = lqr(A,B,Qq,Ru);

K(:,1) = 0;

%% SIMULATION
q0 = [0; pi/18; 0; 0];
%options = odeset('Events',@(t,X)robot_stopped(t,X,p));

[T,X] = ode45(@(t,X)sim_invPendulum(t,X,p),[0 simTime],q0);

% Output States
xW = X(:,1); 
theta1 = X(:,2);
dxW = X(:,3); 
dtheta1 = X(:,4); 

%% ANIMATION
animatePendulum(T,X,p);

%% RECALCULATE TORQUES

%% GRAPHING 
limx = simTime;

figure(1);
subplot(1,2,1)
plot(T,X(:,1:2));
%hold on;
%plot(T,desiredState);
title('Positions')
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


