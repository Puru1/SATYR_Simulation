%%
clc;
clear all;
close all;

global g mW mR IW IR L1 L2 R
g = 9.81;
mW = 0.5;
mR = 2;
L1 = 0.5; 
L2 = .5;
R = 0.05;
IW = mW*R^2;
IR = mR*(2*L2)^2/12; % change for robot

q0 = [0; pi/4; -pi/4; 0; 0; 0];
[T,X] = ode45(@SimpleSegway,[0 10],q0);
plot(T,X)
xW = X(:,1); 
theta1 = X(:,2); 
theta2 = X(:,3);
dxW = X(:,4); 
dtheta1 = X(:,5); 
dtheta2 = X(:,6);
legend("xW","theta1", "theta2", "dxW", "dtheta1", "dtheta2"); 

%%
%ANIMATION
dtA = 0.01;
timeA = 0:dtA:T(end);
ang = 0:0.05:2*pi;
ang_square = (1/16:1/8:1)'*2*pi;

% v = VideoWriter('video.avi');
% open(v)
f = figure;
set(f, 'doublebuffer', 'on');
for k = 1:length(timeA) 
    
    %Robot State
    XW = interp1q(T,xW,timeA(k));
    THETA1 = interp1q(T,theta1,timeA(k));
    THETA2 = interp1q(T,theta2,timeA(k));
    X1 = XW + L1*sin(THETA1);
    XR = X1 + L2*sin(THETA2);
    Z1 = L1*cos(THETA1);
    ZR = Z1 + L2*cos(THETA2);
    
    plot([XW X1],[0 Z1],'k','LineWidth',4)
    hold on
    plot([X1 XR],[Z1 ZR],'k','LineWidth',4);
    fill(XR+2*R*sin(ang_square), ZR+2*R*cos(ang_square),'red');
    hold on
    plot([-1 1],[-R -R],'k','LineWidth',2)
    fill(XW+R*sin(ang),R*cos(ang),[0 0 1])
    axis([-1 1 -1 1])
    
    grid on
    hold off
    F = getframe(f);
    drawnow;    
%     writeVideo(v,F)
end
% close(v)

    
    







