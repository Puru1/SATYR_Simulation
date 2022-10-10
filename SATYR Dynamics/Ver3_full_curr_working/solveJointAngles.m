function [theta1_out,theta2_out] = solveJointAngles(theta,heightScalingFactor,M,L,method)
    
    % Function passed values
%     L = [.16, .2, -.01884, .07329];
%     M = [.66162, .9061, 5.63];
%     theta1 = deg2rad(10)
    
    % Method
    %       1: Automatically solve for both angles using height parameter
    %       2: Given theta1 we assume theta 2 = -1.5*theta1. No control on
    %       height.

    % Solver
%     syms theta2 theta3 theta2_b
    syms theta3 theta2 theta1 theta2_b
    xW = 0;
    L1 = L(1);
    L2 = L(2);
    mCM1 = M(1);
    mCM2 = M(2);
    m1 = mCM1;
    m2 = mCM2;
    mR = M(3);
    
%     theta3 = theta;
    theta3 = -theta2 - theta1;
      
    % CoM calculation (sanity check)
    cm1_pos = fnc_PosCM1([0,0,0,0],L);
    cm2_pos = fnc_PosCM2([0,0,0,0],L);
    cmT_pos = fnc_PosCoM_R([0,0,0,0],L);
    CoMz = (m1*cm1_pos(3) + m2*cm2_pos(3) + mR*cmT_pos(3))/(m1+m2+mR);
    CoMx = (m1*cm1_pos(1) + m2*cm2_pos(1) + mR*cmT_pos(1))/(m1+m2+mR);
    
    % Height Specification
    maxHeight = CoMz;
    des_height = heightScalingFactor*maxHeight;
    h1 = des_height;
    
    if method == 2        
%         theta2 = -2.235*theta1; % hand tuned scaling factor 
        theta2 = -1.5*theta1;
        theta2_t = double(theta2);
    end

    % Solving nonlinear trigonometric system of eq.
    %E1 = z constraint on height 
    %E2 = x constraint for CoM above xW
    
    E1 = (mCM2*((9*L2*cos(theta1 + theta2))/10 + L1*cos(theta1)) + mR*((7329*cos(theta1 + theta2 + theta3))/100000 + (471*sin(theta1 + theta2 + theta3))/25000 + L2*cos(theta1 + theta2) + L1*cos(theta1)) + (2169*L1*mCM1*cos(theta1))/10000)/(mCM1 + mCM2 + mR) - h1 == 0;
    E2 = (mCM1*(xW + (2169*L1*sin(theta1))/10000) + mCM2*(xW + (9*L2*sin(theta1 + theta2))/10 + L1*sin(theta1)) + mR*(xW - (471*cos(theta1 + theta2 + theta3))/25000 + (7329*sin(theta1 + theta2 + theta3))/100000 + L2*sin(theta1 + theta2) + L1*sin(theta1)))/(mCM1 + mCM2 + mR) == xW;
%     E3 = theta1 + theta2 + theta3 == 0;
    
    if method == 1
%         [theta2_t,theta3_t] = vpasolve(E1,E2,theta2,theta3,[-1.5,1.5]); % vpasolve used because symbolic exp could not be found
%         [theta1_t,theta2_t] = vpasolve(E1,E2,theta1,theta2,[-.5,.5],'Random',true); % vpasolve used because symbolic exp could not be found
    [theta1_t,theta2_t] = vpasolve(E1,E2,theta1,theta2,[-1,1], 'Random',true);
    end
    
    if method == 2
        theta3_t = vpasolve(E2,theta3);
    end 
    
%     theta2_out = double(simplify(vpa(theta2_t(1))));
%     theta3_out = double(simplify(vpa(theta3_t(1))));
%     theta2_deg = rad2deg(theta2_out)
%     theta3_deg = rad2deg(theta3_out)

    theta2_out = double(simplify(vpa(theta2_t(1))));
    theta1_out = double(simplify(vpa(theta1_t(1))));
    
    theta2_deg = rad2deg(theta2_out)
    theta1_deg = rad2deg(theta1_out)
end



