function animateCartPole(T,X,p)

    %Param and state expansions
    L1 = p.L1; 
    R = p.R;

    xW = X(:,1); 
    theta1 = X(:,2);

    dtA = 0.01;
    timeA = 0:dtA:T(end);
    ang = 0:0.05:2*pi;
    ang_square = (1/16:1/8:1)'*2*pi;

    f = figure;
    set(f, 'doublebuffer', 'on');

    for k = 1:length(timeA) 

        %L1 = (.35329 - .141)*abs(sin(timeA(k))) + .141;
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
        r = rectangle('Position', [XW-0.075 0 0.15 0.075], 'FaceColor', [0 0 0]);
        axis([-.5 .5 -.05 .5]);
        daspect(ones(1,3));
        grid on
        hold off

        drawnow;    
    end
end