function animatingRobot(T,X,p,tao)

L1 = p.valL.L1; 
L2 = p.valL.L2; 
L3 = p.valL.L3; 
R = p.R; 
L = [L1,L2,L3,R];

dtA = 0.01;
timeA = T(1):dtA:T(end);

XW = interp1(T,X(:,1),timeA);
THETA1 = interp1(T,X(:,2),timeA);
THETA2 = interp1(T,X(:,3),timeA);
THETA3 = interp1(T,X(:,4),timeA);
DXW = interp1(T,X(:,5),timeA);
DTHETA1 = interp1(T,X(:,6),timeA);
DTHETA2 = interp1(T,X(:,7),timeA);
DTHETA3 = interp1(T,X(:,8),timeA);
TAU_W = interp1(T,tao(:,1),timeA);
TAU_K = interp1(T,tao(:,2),timeA);
TAU_H = interp1(T,tao(:,3),timeA);


f = figure(99);
f.WindowState = 'fullscreen';
graph_q = subplot(3,3,3);
graph_dq = subplot(3,3,6);
graph_u = subplot(3,3,9);
graph_grf = subplot(3,3,[7,8]);
graph_robot = subplot(3,3,[1,2,4,5]);

graph_grf.XLim = [T(1) T(end)];
graph_grf.YLim = [-10 10];
set(get(graph_grf,'Title'), 'String', 'GRFz');

set(f, 'doublebuffer', 'on');

if p.captureVideoEnable 
    v = VideoWriter('video_1_21_2021_T2.mp4','MPEG-4');
    open(v)
end

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
    
    % Subplotting
    plot(graph_q,timeA(1:k),XW(1:k),'r',...
        timeA(1:k),THETA1(1:k),'g',...
        timeA(1:k),THETA2(1:k),'b',...
        timeA(1:k),THETA3(1:k),'y');
    graph_q.XLim = [T(1) T(end)];
    graph_q.YLim = [-1 1];
    set(get(graph_q,'Title'), 'String', 'Position [m]');

    plot(graph_dq,timeA(1:k),DXW(1:k),'r',...
        timeA(1:k),DTHETA1(1:k),'g',...
        timeA(1:k),DTHETA2(1:k),'b',...
        timeA(1:k),DTHETA3(1:k),'y');
    graph_dq.XLim = [T(1) T(end)];
    graph_dq.YLim = [-1 1];
    set(get(graph_dq,'Title'), 'String', 'Velocity [m/s]');

    plot(graph_u,timeA(1:k),TAU_W(1:k),'r',...
        timeA(1:k),TAU_K(1:k),'g',...
        timeA(1:k),TAU_H(1:k),'b');
    graph_u.XLim = [T(1) T(end)];
    graph_u.YLim = [-15 15];
    set(get(graph_u,'Title'), 'String', 'Torque [N/m]');

    hold off
    F = getframe(f);
    drawnow;   

    if p.captureVideoEnable == true 
        writeVideo(v,F)
    end

end

    if p.captureVideoEnable == true 
        close(v)
    end
    
end