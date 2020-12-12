%ODE 45 sim
clear all;
close all;

x_initial = [0;pi/2;0;0]; %Initial state vector
Nt = 101;                 %Step Size of time
ti = 0;                   %Initial time (sec)
tf = 35;                  %Final time (sec)
t = linspace(ti,tf,Nt);
%options = odeset('Mass',@mass);
[t1,y1] = ode45(@ode_45_function,t, x_initial);
figure(1);
plot(t1, y1(:,1));
hold on;
plot(t1, y1(:,2));
xlabel('Time (s)');
ylabel('Theta angle (radians)');
legend('theta_1','theta_2');
%%
clf
figure(2)

%Initialize video
myVideo = VideoWriter('TwoLinkPendulum'); %open video file
myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
open(myVideo)

for i=1:max(size(y1))
    color = i*ones(1,3)/101;
    plot(2*sin(y1(i,1)), -2*cos(y1(i,1)), 'o', 'color', color)
    hold on
    plot(2*sin(y1(i,1))+2*sin(y1(i,2)), -2*cos(y1(i,1))-2*cos(y1(i,2)), 'o', 'color', color)
    hold on
    daspect(ones(3,1))
    xlim([-4,4])
    ylim([-5,1])
    
    pause(0.03)
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
end
close(myVideo)

