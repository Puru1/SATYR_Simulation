%% File params
clc;
clear all;
close all;

global K 
captureVideoEnable = false;
simTime = .1;


%% State Space

A = [[ 0,                                  0,                                  0,                                   0, 1, 0, 0, 0]
[ 0,                                  0,                                  0,                                   0, 0, 1, 0, 0]
[ 0,                                  0,                                  0,                                   0, 0, 0, 1, 0]
[ 0,                                  0,                                  0,                                   0, 0, 0, 0, 1]
[ 0, -366.24833494630366698067394275018, -188.39410590419966210281974567255,   0.9234099887471801578511605924744, 0, 0, 0, 0]
[ 0,  740.18456706997409978615597360125, -5.5105711332856315326347449404729, 0.027009955559674137965608159619953, 0, 0, 0, 0]
[ 0,   50.48459523033278680932052516121,  801.73683421276460676817782520071,  -118.87648337458152110937200488042, 0, 0, 0, 0]
[ 0, -802.78112863456403796571959352286,  -808.4233565702798166842598970541,   479.91400738483649677654353531962, 0, 0, 0, 0]];

B = [[                                  0,                                   0,                                  0]
[                                  0,                                   0,                                  0]
[                                  0,                                   0,                                  0]
[                                  0,                                   0,                                  0]
[ -29.409863800236617704041331212731, -0.32988705422470993839771399450997,  5.2457011986366872604429030216691]
[  68.645555799752821054303984606686,  -20.429982895413806860977600637198, 0.15343797227790534160956616511876]
[ -17.131112353166707660124562255118,   45.553917299855688117829455492973, -35.312720164313846130688314913024]
[ -52.303574014088964350870338536456,  -35.312720164313846130688314913024,  76.292236882477666330265350571722]];

Qq = diag([1000 1 10 1 10 10 100 100]);
Ru = diag([1 1 1]);
K = lqr(A,B,Qq,Ru);

q0 = [0; pi/10; -pi/10; 0; 0; 0; 0; 0];
[T,X] = ode45(@SimpleSegway,[0 simTime],q0);


%Are these states errors are actual angles ? 

xW = X(:,1); 
theta1 = X(:,2);
theta2 = X(:,3); 
theta3 = X(:,4);
dxW = X(:,5); 
dtheta1 = X(:,6); 
dtheta2 = X(:,7);
dtheta3 = X(:,8);

tao = zeros(length(xW),3);
for k = 1:length(xW)
     %X_ref = [0 pi/6 -(5*pi)/12 1.5421953896773856832653941961984 0 0 0 0]'; % Theta1 given. Found theta2 via COM calculations for stable point.
     X_ref = [0 0 0 0 0 0 0 0]';
     this_X = X(k,:)';
     this_X = this_X - X_ref;
     torque = -K*this_X;
     tao(k,:) = torque';
end

%% ANIMATION
if captureVideoEnable
    
    dtA = 0.01;
    timeA = 0:dtA:T(end);
    ang = 0:0.05:2*pi;
    ang_square = (1/16:1/8:1)'*2*pi;

    f = figure;
    set(f, 'doublebuffer', 'on');

    v = VideoWriter('video_Refined1_02_06.avi');
    open(v)

    for k = 1:length(timeA) 

        %Robot State
        XW = interp1q(T,xW,timeA(k));
        THETA1 = interp1q(T,theta1,timeA(k));
        THETA2 = interp1q(T,theta2,timeA(k));
        THETA3 = interp1q(T,theta3,timeA(k));
        X1 = XW + L1*sin(THETA1);
        X2 = X1 + L2*sin(THETA2+THETA1);
        XR = X2 + L3*sin(THETA3+THETA2+THETA1);
        Z1 = L1*cos(THETA1);
        Z2 = Z1 + L2*cos(THETA2+THETA1);
        ZR = Z2 + L3*cos(THETA3+THETA2+THETA1);

        plot([XW X1],[0 Z1],'k','LineWidth',4)
        hold on
        plot([X1 X2],[Z1 Z2],'k','LineWidth',4);
        hold on 
        plot([X2 XR],[Z2 ZR],'k','LineWidth',4);
        fill(XR+2*R*sin(ang_square), ZR+2*R*cos(ang_square),'red');
        hold on
        plot([-1 1],[-R -R],'k','LineWidth',2)
        fill(XW+R*sin(ang),R*cos(ang),[0 0 1])
        axis([-2 2 -2 2]);
        daspect(ones(1,3));

        grid on
        hold off
        F = getframe(f);
        drawnow;   

        if captureVideoEnable == true 
            writeVideo(v,F)
        end
    end
    close(v)

end 

    
%% Graphing 

limx = simTime;

% desiredState = ones(length(T), 4);
% desiredState(:,1) = 0 * desiredState(:,1);
% desiredState(:,2) = angle_of_linearization * desiredState(:,2);
% desiredState(:,3) = -angle_of_linearization * desiredState(:,3);
% desiredState(:,4) = 0 * desiredState(:,4);


figure(1);
plot(T,X(:,1:4));
hold on;
%plot(T,desiredState);
title('Positions')
xlim([0 limx]);
xlabel('Time (sec)');
ylabel('Meters or Radians');
legend('xW', '\theta_{1}', '\theta_{2}', '\theta_{3}');

figure(2);
plot(T,X(:,5:8));
title('Velocity')
xlim([0 limx]);
xlabel('Time (sec)');
ylabel('Rads/sec');
name = legend('$\dot{xW}$', '$\dot{\theta_{1}}$', '$\dot{\theta_{2}}$', '$\dot{\theta_{3}}$');
set(name,'Interpreter','latex');

figure(3);
plot(T, tao);
title('Torques')
xlim([0 limx]);
xlabel('Time (sec)');
ylabel('Torque (N*m)');
legend("tao_{1}", "tao_{2}", "tao_{3}"); 




