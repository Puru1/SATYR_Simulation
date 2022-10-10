function [theta1_out,theta2_out] = solveJointAngles(method,l_h,p);
    
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
    L_com = .0733;  
    alpha = l_h;
    
    % method 1 : The more general trigonometric system of eq
    if(method == 1)
        beta = 0;
        z = (L1^2 - L2^2 - (alpha^2 + beta^2))/(-2*L2);
        A = sqrt(alpha^2 + beta^2);
        phi = asin(alpha/A);
        theta2_out = asin(z/A) - phi;
        theta1_out = acos((alpha- L2*cos(theta2_out))/L1);
        
    % method 2: Assuming knowledge about our system we simplify
    elseif (method == 2)
        theta1_out = acos(l_h/(L1+L2+L_com));
        theta2_out = 0;
    end 
%     theta2_deg = rad2deg(theta2_out)
%     theta1_deg = rad2deg(theta1_out)
end



