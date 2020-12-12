function animatePendulum(T,X,p)

global captureVideoEnable 

L1 = p.L1; 

xW = X(:,1); 
theta1 = X(:,2);
R = p.R;


dtA = 0.01;
timeA = 0:dtA:T(end);
ang = 0:0.05:2*pi;
ang_square = (1/16:1/8:1)'*2*pi;

f = figure;
set(f, 'doublebuffer', 'on');

if captureVideoEnable 
    v = VideoWriter('video_Refined2_03_12', 'MPEG-4');
    %v.FrameRate = fps;
    v.Quality = 100;
    open(v)
end

for k = 1:length(timeA) 
    
    %Robot State
    XW = interp1q(T,xW,timeA(k));
    THETA1 = interp1q(T,theta1,timeA(k));
    XR = XW + L1*sin(THETA1);
    ZR = L1*cos(THETA1);

    plot([XW XR],[0 ZR],'k','LineWidth',2);
    hold on;
    fill(XR + .2*R*sin(ang_square), ZR + .2*R*cos(ang_square),'red');
    hold on
    plot([-5 5],[-R -R],'k','LineWidth',2)
    fill(XW+ .2*R*sin(ang), .2*R*cos(ang),[0 0 1]);
    axis([-.5 .5 -.05 .2]);
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