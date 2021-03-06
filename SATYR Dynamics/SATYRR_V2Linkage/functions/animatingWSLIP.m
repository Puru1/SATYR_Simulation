function animatingWSLIP(T,X,p)

R = p.R;
L = p.L;

dtA = 0.01;
timeA = 0:dtA:T(end);
ang = 0:0.05:2*pi;
ang_square = (1/16:1/8:1)'*2*pi;


%Robot State
% XW = interp1(T,X(:,1),timeA);
% LH = interp1(T,X(:,2),timeA);
% THETA1 = interp1(T,X(:,3),timeA);

f = figure(99);
f.WindowState = 'fullscreen';
graph_reducedMod = subplot(1,2,1);
graph_fullMod = subplot(1,2,2);

graph_reducedMod.XLim = [-.5 .5];
graph_reducedMod.YLim = [0 1];

% ax=axes('Parent',graph_fullMod);

set(f, 'doublebuffer', 'on');

if p.captureVideoEnable 
    v = VideoWriter('video_Refined2_03_12', 'MPEG-4');
    %v.FrameRate = fps;
    v.Quality = 100;
    open(v)
end

for k = 1:length(timeA) 
    k
 
    XW = interp1q(T,X(:,1),timeA(k));
    LH = interp1q(T,X(:,2),timeA(k));
    THETA1 = interp1q(T,X(:,3),timeA(k));

    XR = XW + LH*sin(THETA1);
    ZR = LH*cos(THETA1);
    
    [theta1_f, theta2_f] = solveJointAngles(THETA1,LH,p);
    q_vis = [XW, THETA1 + vpa(theta1_f), vpa(-theta1_f+theta2_f),-vpa(theta2_f)];
    SATYRR_Visualize(q_vis,L,graph_fullMod);
    
    % Plotting Reduced order model
    plot(graph_reducedMod,[XW XR],[0 ZR],'k','LineWidth',2);
    hold on;
    fill(graph_reducedMod,XR + .2*R*sin(ang_square), ZR + .2*R*cos(ang_square),'red');
    plot(graph_reducedMod,[-.5 .5], [-R -R],'k','LineWidth',2)
    fill(graph_reducedMod, XW+ .2*R*sin(ang), .2*R*cos(ang),[0 0 1]);
    daspect(ones(1,3));
    grid on
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