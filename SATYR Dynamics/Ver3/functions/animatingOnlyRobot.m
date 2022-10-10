function animatingOnlyRobot(T,X,p)

L1 = p.valL.L1; 
L2 = p.valL.L2; 
L3 = p.valL.L3; 
R = p.R; 
L = [L1,L2,L3,R];

dtA = 0.05;
timeA = T(1):dtA:T(end);

XW = interp1(T,X(:,1),timeA);
THETA1 = interp1(T,X(:,2),timeA);
THETA2 = interp1(T,X(:,3),timeA);
THETA3 = interp1(T,X(:,4),timeA);
DXW = interp1(T,X(:,5),timeA);
DTHETA1 = interp1(T,X(:,6),timeA);
DTHETA2 = interp1(T,X(:,7),timeA);
DTHETA3 = interp1(T,X(:,8),timeA);

f = figure(99);
graph_robot = subplot(1,1,1);

set(f, 'doublebuffer', 'on');

for k = 1:length(timeA) 
%     XW = interp1q(T,xW,timeA(k));
%     THETA1 = interp1q(T,theta1,timeA(k));
%     THETA2 = interp1q(T,theta2,timeA(k));
%     THETA3 = interp1q(T,theta3,timeA(k));
%     DXW = interp1q(T,dxW,timeA(k));
%     DTHETA1 = interp1q(T,dtheta1,timeA(k));
%     DTHETA2 = interp1q(T,dtheta2,timeA(k));
%     DTHETA3 = interp1q(T,theta3,timeA(k));
    xW = XW(k);
    theta1 = THETA1(k);
    theta2 = THETA2(k);
    theta3 = THETA3(k);
    dxW = DXW(k);
    dtheta1 = DTHETA1(k);
    dtheta2 = DTHETA2(k);
    dtheta3 = DTHETA3(k);
    q = [xW,theta1,theta2,theta3];
    dq = [dxW,dtheta1,dtheta2,dtheta3];
    SATYRR_Visualize(q,L,graph_robot);

    hold off
    F = getframe(f);
    drawnow;   

end

end