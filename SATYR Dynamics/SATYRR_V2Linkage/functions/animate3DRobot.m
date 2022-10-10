function animate3DRobot(T,X)
    
L1 = 0.5; 
L2 = 0.5;
L3 = .1;
R = 0.05;

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
for k = 1:length(timeA) 
    XW = interp1q(T,xW,timeA(k));
    YW1_right = .5;
    THETA1 = interp1q(T,theta1,timeA(k));
    THETA2 = interp1q(T,theta2,timeA(k));
    THETA3 = interp1q(T,theta3,timeA(k));
    X1 = XW + L1*sin(THETA1);
    Y1_right = .5;
    X2 = X1 + L2*sin(THETA2+THETA1);
    Y2_right = .5;
    XR = X2 + L3*sin(THETA3+THETA2+THETA1);
    YR_right = .5;
    Z1 = L1*cos(THETA1);
    Z2 = Z1 + L2*cos(THETA2+THETA1);
    ZR = Z2 + L3*cos(THETA3+THETA2+THETA1);

    %Plot right hand side of robot
    plot3([XW X1],[YW1_right Y1_right], [0 Z1],'k','LineWidth',4)
    hold on
    plot3([X1 X2],[Y1_right Y2_right], [Z1 Z2],'k','LineWidth',4);
    hold on 
    plot3([X2 XR],[Y2_right YR_right], [Z2 ZR],'k','LineWidth',4);
    fill3(XR+2*R*sin(ang_square), YR_right+2*R*cos(ang_square), ZR+2*R*cos(ang_square),'red');
    hold on
    %plot3([-1 1],[-R -R], [-R -R],'k','LineWidth',2)
    fill3(XW+R*sin(ang),YW1_right + R*cos(ang),R*cos(ang), [0 0 1])
    
    %Plot left hand side of robot
    plot3([XW X1],[-YW1_right -Y1_right], [0 Z1],'k','LineWidth',4)
    hold on
    plot3([X1 X2],[-Y1_right -Y2_right], [Z1 Z2],'k','LineWidth',4);
    hold on 
    plot3([X2 XR],[-Y2_right -YR_right], [Z2 ZR],'k','LineWidth',4);
    fill3(XR+2*R*sin(ang_square), -YR_right+2*R*cos(ang_square), ZR+2*R*cos(ang_square),'red');
    hold on
    fill3(XW+R*sin(ang),-YW1_right + R*cos(ang),R*cos(ang), [0 0 1])
    
    %Plot connection between hips
    plot3([X2 XR],[-YR_right YR_right], [Z2 ZR],'k','LineWidth',4)

    axis([-2 2 -2 2]);
    daspect(ones(1,3));

    grid on
    hold off
    drawnow;    
end

end