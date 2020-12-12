clc;
clear all;
close all;

global g mW mR IW IR h R K
g = 9.81;
mW = 1;
mR = 2;
h = 0.5;
R = 0.05;
IW = mW*R^2;
IR = mR*(2*h)^2/12;

%State Space
A = [[0 -(R^2*g*h^2*mR^2)/(IR*IW + IR*R^2*mR + IR*R^2*mW + IW*h^2*mR + R^2*h^2*mR*mW) 0 0]
     [0 (g*h*mR*(IW + R^2*mR + R^2*mW))/(IR*IW + IR*R^2*mR + IR*R^2*mW + IW*h^2*mR + R^2*h^2*mR*mW) 0 0]];
A = [[zeros(2,2) eye(2)]; A];
B = [(R*(mR*h^2 + R*mR*h + IR))/(IR*IW + IR*R^2*mR + IR*R^2*mW + IW*h^2*mR + R^2*h^2*mR*mW);
     -(IW + R^2*mR + R^2*mW + R*h*mR)/(IR*IW + IR*R^2*mR + IR*R^2*mW + IW*h^2*mR + R^2*h^2*mR*mW)];
B = [0; 0; B];
w = (g/h)^0.5;
Qq = diag([1 1/R 1/w 1/(w*R)]);
Ru = 1;
K = lqr(A,B,Qq,Ru);

q0 = [0.1; pi/10; 0; 0];
[T,X] = ode45(@SimpleSegway,[0 10],q0);
subplot(2,1,1),plot(T,X), grid on
xW = X(:,1); 
theta = X(:,2); 
dxW = X(:,3); 
dtheta = X(:,4); 
tau = -K*X';
subplot(2,1,2), plot(T,tau), grid on

%------------------------------------------------------------------------
%ANIMATION
dtA = 0.05;
timeA = 0:dtA:T(end);
ang = 0:0.05:2*pi;

% v = VideoWriter('video.avi');
% open(v)
f = figure;
set(f, 'doublebuffer', 'on');
%Initialize video
myVideo = VideoWriter('TwoLinkInvertedPendulum'); %open video file
myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
open(myVideo)
for k = 1:length(timeA) 
    
    %Robot State
    XW = interp1q(T,xW,timeA(k));
    THETA = interp1q(T,theta,timeA(k));
    XR = XW + h*sin(THETA);
    ZR = h*cos(THETA);
    theta_wheel = XW/R;
    
    plot([XW XR],[0 ZR],'k','LineWidth',4)
    hold on
    plot([-1 1],[-R -R],'k','LineWidth',2)
    fill(XW+R*sin(ang),R*cos(ang),[0 0 1])
    plot([XW XW+R*sin(theta_wheel)],[0 R*cos(theta_wheel)],'r','LineWidth',2)
    axis([-1 1 -1 1])
    
    grid on
    hold off
    drawnow;
    
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
end
close(myVideo);

    
    







