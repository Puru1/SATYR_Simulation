function [theta1_out,theta2_out] = solveJointAngles(theta1_r,l_h,p);
    
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
    syms theta1 theta2 theta2_b real
    
    L1 = p.valL.L1;
    L2 = p.valL.L2;
    L3 = p.valL.L3;
    
    zPend = vpa(l_h*cos(theta1_r));
    xPend = vpa(l_h*sin(theta1_r));
    
    alpha = l_h - L3;
    beta = 0;
    z = (L1^2 - L2^2 - (alpha^2 + beta^2))/(-2*L2);
    A = sqrt(alpha^2 + beta^2);
    phi = asin(alpha/A);
    theta2_out = asin(z/A) - phi;
    theta1_out = acos((alpha- L2*cos(theta2_out))/L1);
    
%     theta2_deg = rad2deg(theta2_out)
%     theta1_deg = rad2deg(theta1_out)
end



