function animatingRobot(p)

    L = [p.valL.L1, p.valL.L2, p.valL.L3];
    pjoint = zeros(3,n); % origin, joint, and end-effector positions in inertial frame
    pCoM = zeros(3,n); % link CoM positions in inertial frame
    pvirtual = zeros(3,2*n);
    
    % plot
    lineWidth = 2;
    scatterSize = 50;
    xlimit = sum(L);
    ylimit = xlimit;
    
    f = figure;
    set(f, 'doublebuffer', 'on');
    
    plotOrigin = scatter(0,0,scatterSize,'k','filled'); % plot inertial frame origin;
    hold on
    plotLink = plot([0,pjoint(1,:)],[0,pjoint(3,:)],'k-','linewidth',lineWidth);
%     plotLinkVirtual = plot([0,pvirtual(1,:)],[0,pvirtual(3,:)],'k--','linewidth',lineWidth);
%     plotJoint = scatter(pjoint(1,1:end - 1),pjoint(3,1:end - 1),scatterSize,'b','filled');
%     plotEndEffector = scatter(pjoint(1,end),pjoint(3,end),scatterSize,'g','filled');
%     plotCoM = scatter(pCoM(1,:),pCoM(3,:),scatterSize,'r','filled');

    hold off
    axis equal
    xlim(xlimit*[-1,1])
    ylim(ylimit*[-0.1,1.2])
    xlabel('x [mm]')
    ylabel('z [mm]')
    legend([plotLink,plotLinkVirtual,plotJoint,plotEndEffector,plotCoM,plotOrigin],{'Link','Virtual Link','Joint','End-Effector','Link CoM','Inertial Frame Origin'},'location','northeast')
    grid on
    axis([-.75 .75 -.75 .75]);
    daspect(ones(1,3));

    grid on
    hold off
    % F = getframe(f);
    drawnow;   

    
lineWidth = 2;
scatterSize = 50;
xlimit = sum(L);
ylimit = xlimit;


    
end