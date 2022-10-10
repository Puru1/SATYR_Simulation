function [theta2_out,theta3_out] = solveJointAngles(theta1, L, height)
    
    syms theta2 theta3
    L1 = L(1);
    L2 = L(2);
    LR = L(3);
    h1 = height;

    E1 = L2*sin(theta2) + LR*sin(theta3) + L1*sin(theta1) == 0;
    E2 = L2*cos(theta2) + LR*cos(theta3) +(-h1 + L1*cos(theta1)) == 0;
    [theta2,theta3] = solve(E1,E2,theta2,theta3)

    theta2_rad = vpa(theta2(1));
    theta3_rad = vpa(theta3(1));

    theta2_out = simplify(rad2deg(theta2_rad))
    theta3_out = simplify(rad2deg(theta3_rad))

end



