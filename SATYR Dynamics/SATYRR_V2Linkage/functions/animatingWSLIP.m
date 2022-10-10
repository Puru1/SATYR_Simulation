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
% graph_reducedMod = subplot(1,2,1);
% graph_fullMod = subplot(1,2,2);
tiledlayout(1,2)
ax1 = nexttile;
ax2 = nexttile;

% ax1.XLim = [-.5 .5];
% ax1.YLim = [0 1];

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
    
    [theta1_f, theta2_f] = solveJointAngles(2,LH,p);
    q_vis = [XW, theta1_f,THETA1]; % THETA1 = IMU pitch angle 
    SATYRR_Visualize(q_vis,L,ax2);
    
    % Plotting Reduced order model
    cla(ax1);
    grid on
    ax1.XLim = [-.5 .5];
    ax1.YLim = [-.1 1];
    hold(ax1,'on')
    plot(ax1,[XW XR],[0 ZR],'k','LineWidth',2);
    fill(ax1 ,XR + .2*R*sin(ang_square), ZR + .2*R*cos(ang_square),'red');
    plot(ax1,[-.5 .5], [-.2*R -.2*R],'k','LineWidth',2)
    fill(ax1, XW+ .2*R*sin(ang), .2*R*cos(ang),[0 0 1]);
    hold(ax1,'off')
    
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