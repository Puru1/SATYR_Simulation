%% FILE PARAMS
clc;
clear all;
close all;

%% FILE FEATURE SETTINGS
simTime = 5;

%% LOCAL VARIBALES
p = getParams();
g = p.g;
h_H = 1.0; % (m)
omega_H = sqrt(g/h_H);
h_R = p.L1;
omega_R = sqrt(g/h_R);

%% STATE SPACE
A = fnc_A();
B = fnc_B();
A = [[zeros(2,2) eye(2)]; A]; 
B = [zeros(2,1); B];   
C = eye(4);
D = 0;
dt = .0010;


%With inital vel
% Qq = diag([(6*10^3) (5.5*10^4) 11 1]); % [xW theta dxW dtheta]
%Qq = diag([(8*10^2) (2.5*10^5) (5*10^3) (9*10^2)]); % [xW theta dxW dtheta]

%Qq = diag([(8*10^2) (1*10^1) (8*10^2) (9*10^2)]); % [xW dxW xCom DCM] w xCoM gain
%Qq = diag([(8*10^3) (5*10^3) (8*10^0) (1.5*10^5)]); % [xW dxW xCom DCM] w/o

%Less aggresive gains
Qq = diag([(8*10^3) (5*10^1) (8*10^0) (1.5*10^4)]); % [xW dxW xCom DCM] w/o


Ru = 1;
p.Qq = Qq;
p.Ru = Ru;

% SWITCH TO DCM COORD.
T = [[1 0 0 0];
     [0 0 1 0];
     [1 h_R 0 0];
     [0 1 0 1/omega_R]];
 
%T = eye(4); 

% Perform state transform to include DCM
A = T*A*(T^-1)
B = T*B;

% Continuous time controller
p.T = T;
K = lqr(A,B,Qq,Ru)

%My ind. set gain.
%For the DCM tracking without a desired position or velocity we must:
%K(1) = 0;
%K(2) = 0;
K(3) = 0;
K
write_fcn_m('fnc_K.m',{},[],{K,'K'});

%% SIMULATION
q0 = [0; .1; 0; 0]; %[x,th,dx,dth];
X_ref = [0;0;0;0]; %[x, dx, xCoM, DCM_R];

%options = odeset('Events',@(t,X)robot_stopped(t,X,p));

%[T,X] = ode45(@(t,X)sim_cartPole(t,X,p,X_ref),[0 simTime],q0);

[T,X,q_ref] = eulerIntCartPole(p,X_ref,simTime,q0,K);
%out = computeTorques(X,p,X_ref); % out(1) -> Fx |  out(2) -> tau

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
animateCartPole(T,X,p);

%% GRAPHING 
limx = 1.3;

figure(1);
subplot(2,2,1)
plot(T,X(:,1:2));
%hold on;
%plot(T,desiredState);
title('Positions')
xlim([0 limx]);
xlabel('Time (sec)');
ylabel('Meters or Radians');
legend('xW', '\theta_{1}');

subplot(2,2,2)
plot(T,X(:,3:4));
title('Velocity')
xlim([0 limx]);
xlabel('Time (sec)');
ylabel('Rads/sec');
name = legend('$\dot{xW}$', '$\dot{\theta_{1}}$');
set(name,'Interpreter','latex');

subplot(2,2,[3 4])
plot(T,(X(:,2) + X(:,4)/omega_R));
hold on;
plot(T,q_ref(:,4));
title('Robot Unitless DCM')
xlim([0 limx]);
xlabel('Time (sec)');
ylabel('Rad');
set(name,'Interpreter','latex');



% figure(2)
% endIndex = 200;
% plot(T(1:endIndex),out(1:endIndex,2));
% title('Applied Wheel Torque');
% xlabel('Time (sec)');
% ylabel('Nm');
% ylim([-20 20]);


