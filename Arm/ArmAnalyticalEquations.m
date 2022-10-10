%% VARIABLE DECLARATION
clc;
clear all;

syms th1 th2 th3 th4 real
syms dth1 dth2 dth3 dth4 real
syms ddth1 ddth2 ddth3 ddth4 real
syms g mSh L_arm L_forearm L_b real

%States
q = [th1; th2; th3; th4];
dq = [dth1; dth2; dth3; dth4];
ddq = [ddth1; ddth2; ddth3; ddth4];

%% Homegeneous transforms and frame init.

% Robot base (B) -> Shoulder M1 (S1)
% rotates around y axis of robot body i.e. rotation axis = [0; 1; 0] 
Ro_BS1 = [[ cos(th1) 0 sin(th1)];
          [   0      1    0    ];
          [-sin(th1) 0 cos(th1)]];
Tr_BS1 = [0; -L_b/2; 0];
HTM_BS1 = [[Ro_BS1  Tr_BS1]
           [0   0   0   1]];
       
% Shoulder M1 (S1) -> Shoulder M2 (S2)
% Rotates around the x axis of robot body i.e. [1; 0; 0];
Ro_S1S2 = [[1     0          0   ];
          [0   cos(th2) -sin(th2)];
          [0   sin(th2)  cos(th2)]];
Tr_S1S2 = [0.00414; -(0.04803+0.05365); 0];
HTM_S1S2 = [[Ro_S1S2 Tr_S1S2]
            [0  0   0   1   ]];

HTM_BS2 = simplify(HTM_BS1 * HTM_S1S2);

% Shoulder M2 (S2) -> Shoulder M3 (S3)
% Rotates around the z axis of the robot body i.e. [0; 0; 1];
Ro_S2S3 = [[cos(th3) -sin(th3) 0];
           [sin(th3)  cos(th3) 0];
           [   0        0      1]];
Tr_S2S3 = [0;0;0];
HTM_S2S3 = [[Ro_S2S3 Tr_S2S3]
            [0   0    0   1 ]];

HTM_BS3 = simplify(HTM_BS2 * HTM_S2S3);

% Shoulder M3 (S3) -> Elbow (E)
% rotates around y axis of robot body i.e. rotation axis = [0; 1; 0] 
Ro_S3E = [[ cos(th4) 0 sin(th4)];
          [   0      1    0    ];
          [-sin(th4) 0 cos(th4)]];
Tr_S3E = [0; 0; -L_arm];
HTM_S3E = [[Ro_S3E  Tr_S3E]
           [0   0   0   1]];
       
HTM_BE = simplify(HTM_BS3 * HTM_S3E);
       
% Elbow (E) -> End effector (eF)
% no rotation and simple translation
Ro_EeF = eye(3);
Tr_EeF = [-0.05118; 0; -L_forearm];
HTM_EeF = [[Ro_EeF Tr_EeF]
           [0   0   0   1]];
 
HTM_BeF = simplify(HTM_BE * HTM_EeF);


%% Jacobians
PosSh_body = HTM_BS1(1:3,4);
PosSh = HTM_BS3(1:3,4);
PosElb = HTM_BE(1:3,4);
PoseF = HTM_BeF(1:3,4);

%% Graphing forearm
close all;
l_b = .15;
l_arm = .11945;
l_forearm = .2115;
theta1 = 0; % angle is flipped
theta2 = 0; % angle is flipped
theta3 = 0; % angle is NOT flipped
theta4 = 0; % angle is flipped

posSh_body = vpa(subs(PosSh_body, [th1 th2 th3 th4 L_arm L_forearm L_b], [theta1 theta2 theta3 theta4 l_arm l_forearm l_b]));
posSh = vpa(subs(PosSh, [th1 th2 th3 th4 L_arm L_forearm L_b], [theta1 theta2 theta3 theta4 l_arm l_forearm l_b]));
posElb = vpa(subs(PosElb, [th1 th2 th3 th4 L_arm L_forearm L_b], [theta1, theta2, theta3, theta4, l_arm l_forearm l_b]));
poseF = vpa(subs(PoseF, [th1 th2 th3 th4 L_arm L_forearm L_b], [theta1, theta2, theta3, theta4 l_arm l_forearm l_b]));
posElb_2 = [poseF(1),poseF(2) ,posElb(3)];

% Plot robot arms
plot3([posSh_body(1), posSh(1), posElb(1), poseF(1)], [posSh_body(2), posSh(2), posElb(2), poseF(2)], [posSh_body(3), posSh(3), posElb(3),  poseF(3)], 'LineWidth', 2, 'Color', [0 0 0]);
hold on;
plot3([posSh(1)], [posSh(2)], [posSh(3)], '.','Color','b','MarkerSize',30);
hold on;
plot3([posElb(1)], [posElb(2)], [posElb(3)], '.','Color','g','MarkerSize',30);
hold on;
plot3([posElb_2(1)], [posElb_2(2)], [posElb_2(3)], '.','Color',[0,0,0],'MarkerSize',30);
hold on;
plot3([poseF(1)], [poseF(2)], [poseF(3)], '.','Color','r','MarkerSize',30);
hold on;

% Dotted Line and extra plot
plot3([posSh_body(1)], [posSh_body(2)], [posSh_body(3)], '.','Color',[0,0,0],'MarkerSize',30);
hold on;
plot3([posElb(1), posElb_2(1), poseF(1)], [posElb(2), posElb_2(2), poseF(2)], [posElb(3), posElb_2(3),  poseF(3)], '--', 'LineWidth', 1, 'Color', [0 0 0]);



labels = {'Shoulder','Elbow','End-Effector'};
text([posSh(1), posElb(1), poseF(1)], [posSh(2), posElb(2), poseF(2)], [posSh(3), posElb(3),  poseF(3)],labels,'VerticalAlignment','bottom','HorizontalAlignment','right')

% Plotting robot body
P = [0,0,0] ;   % you center point 
L = [l_b,l_b,.2] ;  % your cube dimensions 
O = P-L/2 ;       % Get the origin of cube so that P is at center 
plotcube(L,O,.3,[0 0 0]);   % use function plotcube 
hold on
plot3(P(1),P(2),P(3),'*k')

grid on;
daspect(ones(1,3))
u = [-1, 1];
xlim([-.3 .3])
ylim([-.4 .2])
zlim([-.4 .2])
view([84, 17])
xlabel('x(t)')
ylabel('y(t)')
zlabel('z(t)')