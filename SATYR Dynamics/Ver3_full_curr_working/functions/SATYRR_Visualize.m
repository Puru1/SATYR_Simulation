function SATYRR_Visualize(th,L,CoM)
% Unit: mm-g-s-rad
% visualize SATYRR as an x-z planar 3-DoF manipulator
% vector index seequence: (1) -> ankle. (2) knee. (3) hip.
% th -> joint positions
% L -> link lengths
% CoM -> CoM positions relatiev to the corresponding JOINT
% Assume ankle joint center as inertial frame origin
n = 3; % # of DoFs
T = zeros(n,4,4);
pjoint = zeros(3,n); % origin, joint, and end-effector positions in inertial frame
pCoM = zeros(3,n); % link CoM positions in inertial frame
pvirtual = zeros(3,2*n);
for iDoF = 1:n
    T(iDoF,:,:) = [Ry(th(iDoF)),[L(iDoF)*sin(th(iDoF)); 0; L(iDoF)*cos(th(iDoF))]; [0,0,0,1]];
    if iDoF > 1
        T(iDoF,:,:) = squeeze(T(iDoF - 1,:,:))*squeeze(T(iDoF,:,:));
    end
    pjoint(:,iDoF) = getp(squeeze(T(iDoF,:,:)));
    pCoM(:,iDoF) = getp(squeeze(T(iDoF,:,:))*[eye(3),CoM(:,iDoF);[0,0,0,1]]);
    pvirtual(:,2*iDoF - 1:2*iDoF) = [pCoM(:,iDoF),pjoint(:,iDoF)];
end

% plot
lineWidth = 2;
scatterSize = 50;
xlimit = sum(L);
ylimit = xlimit;

figure(99)
pjoint
pvirtual
pCom
plotOrigin = scatter(0,0,scatterSize,'k','filled'); % plot inertial frame origin;
hold on
plotLink = plot([0,pjoint(1,:)],[0,pjoint(3,:)],'k-','linewidth',lineWidth);
plotLinkVirtual = plot([0,pvirtual(1,:)],[0,pvirtual(3,:)],'k--','linewidth',lineWidth);
plotJoint = scatter(pjoint(1,1:end - 1),pjoint(3,1:end - 1),scatterSize,'b','filled');
plotEndEffector = scatter(pjoint(1,end),pjoint(3,end),scatterSize,'g','filled');
plotCoM = scatter(pCoM(1,:),pCoM(3,:),scatterSize,'r','filled');

hold off
axis equal
xlim(xlimit*[-1,1])
ylim(ylimit*[-0.1,1.2])
xlabel('x [mm]')
ylabel('z [mm]')
legend([plotLink,plotLinkVirtual,plotJoint,plotEndEffector,plotCoM,plotOrigin],{'Link','Virtual Link','Joint','End-Effector','Link CoM','Inertial Frame Origin'},'location','northeast')
grid on

end

function ry = Ry(th)
ry = [cos(th),0,sin(th); 0,1,0; -sin(th),0,cos(th)];
end

function p = getp(T)
p = T(1:3,end);
end