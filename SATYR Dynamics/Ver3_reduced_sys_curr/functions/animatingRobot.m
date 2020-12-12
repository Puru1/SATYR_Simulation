function animatingRobot(T,X,p)

global captureVideoEnable 
L1 = p.valL.L1; 
L2 = p.valL.L2; 
L3 = p.valL.L3; 
R = p.R; 

xW = X(:,1); 
theta1 = X(:,2);
theta2 = X(:,3);
theta3 = X(:,4);

dtA = 0.01;
timeA = 0:dtA:T(end);
ang = 0:0.05:2*pi;
ang_square = (1/16:1/8:1)'*2*pi;

f = figure;
set(f, 'doublebuffer', 'on');

if captureVideoEnable 
    v = VideoWriter('video_brokenSim_06_07', 'MPEG-4');
    v.Quality = 100;
    open(v)
end

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
    plot([-5 5],[-R -R],'k','LineWidth',2)
    fill(XW+R*sin(ang),R*cos(ang),[0 0 1])
    axis([-2.5 2.5 -.75 .75]);
    daspect(ones(1,3));

    grid on
    hold off
    F = getframe(f);
    drawnow;   

    if captureVideoEnable == true 
        writeVideo(v,F)
    end

end

    if captureVideoEnable == true 
        close(v)
    end
    
end