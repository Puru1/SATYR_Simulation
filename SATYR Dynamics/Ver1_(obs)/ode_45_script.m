%ODE 45 sim
clear all;

x_initial = [0;0;pi;0;0;0;0;0;0;0]; %Initial state vector
Nt = 101;                 %Step Size of time
ti = 0;                   %Initial time (sec)
tf = 35;                  %Final time (sec)
t = linspace(ti,tf,Nt);
%options = odeset('Mass',@mass);
[t1,y1] = ode45(@ode_45_sim,t, x_initial);
figure(1);
plot(t1, y1(:,2));
hold on;
plot(t1, y1(:,3));
hold on;
plot(t1, y1(:,4));
hold on;
plot(t1, y1(:,5));
xlabel('Time (s)');
ylabel('Theta angle (radians)');
legend('phi_w','theta_1','theta_2','theta_3');
%%
%clf
%figure(2)
%for i=1:max(size(y1))
%    color = i*ones(1,3)/101;

%    plot(2*sin(y1(i,1)), -2*cos(y1(i,1)), 'o', 'color', color)
%    hold on
%    plot(2*sin(y1(i,1))+2*sin(y1(i,2)), -2*cos(y1(i,1))-2*cos(y1(i,2)), 'o', 'color', color)
%pause(0.03)
%hold on
%daspect(ones(3,1))
%xlim([-4,4])
%ylim([-5,1])
%end
