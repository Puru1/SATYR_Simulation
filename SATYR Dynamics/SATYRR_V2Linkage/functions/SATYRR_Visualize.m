<<<<<<< HEAD
<<<<<<< HEAD
function SATYRR_Visualize(q, L,f)
% Unit: mm-g-s-rad
% visualize SATYRR as an x-z planar 3-DoF manipulator
% vector index seequence: (1) -> ankle. (2) knee. (3) hip.
% q -> joint positions
% L -> link lengths
% CoM -> CoM positions relatiev to the corresponding JOINT

n = 3; % # of DoFs
pjoint = zeros(3,n); % origin, joint, and end-effector positions in inertial frame
pCoM = zeros(3,n); % link CoM positions in inertial frame
pvirtual = zeros(3,2*n);

posCm1 = fnc_PosCM1(q,L);
posCm2 = fnc_PosCM2(q,L);
posCmR = fnc_PosCoM_R(q,L);

posW = q(1);
posK = fnc_PosK(q,L);
posH = fnc_PosH(q,L);
posT = fnc_PosT(q,L);

pjoint(:,1) = posK;
pjoint(:,2) = posH;
pjoint(:,3) = posT;

pCoM(:,1) = posCm1;
pCoM(:,2) = posCm2;
pCoM(:,3) = posCmR;

% CoM Calc
M = [.66162, .9061, 5.63];
m1 = M(1);
m2 = M(2);
mR = M(3);
CoMz = (m1*posCm1(3) + m2*posCm2(3) + mR*posCmR(3))/(m1+m2+mR);
CoMx = (m1*posCm1(1) + m2*posCm2(1) + mR*posCmR(1))/(m1+m2+mR);
CoM = [CoMx;0;CoMz];

for iDoF = 1:n
    pvirtual(:,2*iDoF - 1:2*iDoF) = [pCoM(:,iDoF),pjoint(:,iDoF)];
end

% plot
lineWidth = 2;
scatterSize = 50;
wheelSize = 4*scatterSize;
xlimit = sum(L);
ylimit = xlimit;

plotWheel = scatter(posW,0,wheelSize,'k','filled'); % plot inertial frame origin;
hold on
plotLink = plot(f,[posW,pjoint(1,:)],[0,pjoint(3,:)],'k-','linewidth',lineWidth);
plotLinkVirtual = plot(f,[posW,pvirtual(1,:)],[0,pvirtual(3,:)],'k--','linewidth',lineWidth);
plotJoint = scatter(f,pjoint(1,1:end - 1),pjoint(3,1:end - 1),scatterSize,'b','filled');
plotEndEffector = scatter(f,pjoint(1,end),pjoint(3,end),scatterSize,'b','filled');
plotCoM = scatter(f,pCoM(1,:),pCoM(3,:),scatterSize,'r','filled');
plotCoM_R = scatter(f,CoM(1,end),CoM(3,end),scatterSize,'g','filled');

hold off
axis equal
xlim(xlimit*[-1,1])
ylim(ylimit*[-0.1,1.2])
xlabel('x [mm]')
ylabel('z [mm]')
legend([plotLink,plotLinkVirtual,plotJoint,plotEndEffector,plotCoM,plotWheel,plotCoM_R],{'Link','Virtual Link','Joint','End-Effector','Link CoM','Inertial Frame Origin','CoM of Robot'},'location','northwest')
grid on

end
=======
function SATYRR_Visualize(q,L,f)
% Unit: mm-g-s-rad
% visualize SATYRR as an x-z planar 3-DoF manipulator
% vector index seequence: (1) -> ankle. (2) knee. (3) hip.
% q -> joint positions
% L -> link lengths
% CoM -> CoM positions relatiev to the corresponding JOINT

n = 3; % # of DoFs
pjoint = zeros(3,n); % origin, joint, and end-effector positions in inertial frame
pCoM = zeros(3,n); % link CoM positions in inertial frame
pvirtual = zeros(3,2*n);

posCm1 = fnc_PosCM1(q,L);
posCm2 = fnc_PosCM2(q,L);
posCmR = fnc_PosCoM_R(q,L);

posW = q(1);
posK = fnc_PosK(q,L);
posH = fnc_PosH(q,L);
posT = fnc_PosT(q,L);

pjoint(:,1) = posK;
pjoint(:,2) = posH;
pjoint(:,3) = posT;

pCoM(:,1) = posCm1;
pCoM(:,2) = posCm2;
pCoM(:,3) = posCmR;

% CoM Calc
M = [.66162, .9061, 5.63];
m1 = M(1);
m2 = M(2);
mR = M(3);
CoMz = (m1*posCm1(3) + m2*posCm2(3) + mR*posCmR(3))/(m1+m2+mR);
CoMx = (m1*posCm1(1) + m2*posCm2(1) + mR*posCmR(1))/(m1+m2+mR);
CoM = [CoMx;0;CoMz];

for iDoF = 1:n
    pvirtual(:,2*iDoF - 1:2*iDoF) = [pCoM(:,iDoF),pjoint(:,iDoF)];
end

% plot
lineWidth = 2;
scatterSize = 50;
wheelSize = 4*scatterSize;
xlimit = sum(L);
ylimit = xlimit;

plotWheel = scatter(posW,0,wheelSize,'k','filled'); % plot inertial frame origin;
hold on
plotLink = plot(f,[posW,pjoint(1,:)],[0,pjoint(3,:)],'k-','linewidth',lineWidth);
plotLinkVirtual = plot(f,[posW,pvirtual(1,:)],[0,pvirtual(3,:)],'k--','linewidth',lineWidth);
plotJoint = scatter(f,pjoint(1,1:end - 1),pjoint(3,1:end - 1),scatterSize,'b','filled');
plotEndEffector = scatter(f,pjoint(1,end),pjoint(3,end),scatterSize,'b','filled');
plotCoM = scatter(f,pCoM(1,:),pCoM(3,:),scatterSize,'r','filled');
plotCoM_R = scatter(f,CoM(1,end),CoM(3,end),scatterSize,'g','filled');

hold off
axis equal
xlim(xlimit*[-1,1])
ylim(ylimit*[-0.1,1.2])
xlabel('x [mm]')
ylabel('z [mm]')
legend([plotLink,plotLinkVirtual,plotJoint,plotEndEffector,plotCoM,plotWheel,plotCoM_R],{'Link','Virtual Link','Joint','End-Effector','Link CoM','Inertial Frame Origin','CoM of Robot'},'location','northwest')
grid on

end
>>>>>>> b6f1ecc8a3fb2eda4e78b6658502c98c386c0207
=======
function SATYRR_Visualize(q, L,f)
% Unit: mm-g-s-rad
% visualize SATYRR as an x-z planar 3-DoF manipulator
% vector index seequence: (1) -> ankle. (2) knee. (3) hip.
% q -> joint positions
% L -> link lengths
% CoM -> CoM positions relatiev to the corresponding JOINT

n = 3; % # of DoFs
pjoint = zeros(3,n); % origin, joint, and end-effector positions in inertial frame
pCoM = zeros(3,n); % link CoM positions in inertial frame
pvirtual = zeros(3,2*n);

posCm1 = fnc_PosCM1(q,L);
posCm2 = fnc_PosCM2(q,L);
posCmR = fnc_PosCoM_R(q,L);

posW = q(1);
posK = fnc_PosK(q,L);
posH = fnc_PosH(q,L);
posT = fnc_PosT(q,L);

pjoint(:,1) = posK;
pjoint(:,2) = posH;
pjoint(:,3) = posT;

pCoM(:,1) = posCm1;
pCoM(:,2) = posCm2;
pCoM(:,3) = posCmR;

% CoM Calc
M = [.66162, .9061, 5.63];
m1 = M(1);
m2 = M(2);
mR = M(3);
CoMz = (m1*posCm1(3) + m2*posCm2(3) + mR*posCmR(3))/(m1+m2+mR);
CoMx = (m1*posCm1(1) + m2*posCm2(1) + mR*posCmR(1))/(m1+m2+mR);
CoM = [CoMx;0;CoMz];

for iDoF = 1:n
    pvirtual(:,2*iDoF - 1:2*iDoF) = [pCoM(:,iDoF),pjoint(:,iDoF)];
end

% plot
lineWidth = 2;
scatterSize = 50;
wheelSize = 4*scatterSize;
xlimit = sum(L);
ylimit = xlimit;

plotWheel = scatter(posW,0,wheelSize,'k','filled'); % plot inertial frame origin;
hold on
plotLink = plot(f,[posW,pjoint(1,:)],[0,pjoint(3,:)],'k-','linewidth',lineWidth);
plotLinkVirtual = plot(f,[posW,pvirtual(1,:)],[0,pvirtual(3,:)],'k--','linewidth',lineWidth);
plotJoint = scatter(f,pjoint(1,1:end - 1),pjoint(3,1:end - 1),scatterSize,'b','filled');
plotEndEffector = scatter(f,pjoint(1,end),pjoint(3,end),scatterSize,'b','filled');
plotCoM = scatter(f,pCoM(1,:),pCoM(3,:),scatterSize,'r','filled');
plotCoM_R = scatter(f,CoM(1,end),CoM(3,end),scatterSize,'g','filled');

hold off
axis equal
xlim(xlimit*[-1,1])
ylim(ylimit*[-0.1,1.2])
xlabel('x [mm]')
ylabel('z [mm]')
legend([plotLink,plotLinkVirtual,plotJoint,plotEndEffector,plotCoM,plotWheel,plotCoM_R],{'Link','Virtual Link','Joint','End-Effector','Link CoM','Inertial Frame Origin','CoM of Robot'},'location','northwest')
grid on

end
>>>>>>> adding_folders
