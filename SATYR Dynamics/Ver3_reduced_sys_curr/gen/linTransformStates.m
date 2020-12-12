function [T,V] = linTransformStates(PosR,PosW,q,dq)

    %% REDECLARATION OF VARIABLES IN USE
    syms xW theta1 theta2 theta3 real %joint positions
    syms dxW dtheta1 dtheta2 dtheta3 real%joint velocities
    syms ddxW ddtheta1 ddtheta2 ddtheta3 real%joint accelerations
    syms tau1 tau2 tau3 real%torques
    syms g mW mCM1 mK mCM2 mH mR IW IK IH IR L1 L2 L3 R real% [gravity, wheel mass, knee mass, hip mass, robot mass, wheel inertia, knee inertia, hip inertia ..., wheel radius]
    syms I0x I0y I0z I1x I1y I1z I2x I2y I2z I3x I3y I3z real

    q = q(2:4);
    dq = dq(2:4);

    %% TRANSFORMATION CALC.
    X = PosR(1) - PosW(1);
    X_d = jacobian(X,q(2:4)') * dq(2:4);

    zR = PosR(3) - PosW(3);
    dzR = jacobian(zR,q(2:4)') * dq(2:4);

    Y = 1/zR;
    dY = -dzR/(zR^2);

    theta_t = atan(X*Y);
    dtheta_t = (1/(1+(X*Y)^2))*(X_d*Y + X*dY);

    %First order approximation for theta_t
    theta_t_derivativeTerm = jacobian(theta_t,[q(2:4); dq(2:4)]);

    theta_t_constantTerm = simplify(subs(theta_t,[theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 L1 L2 L3],[theta1_num theta2_num 0 0 0 0 .5 .5 .1]));
    theta_t_derivativeTerm = simplify(subs(theta_t_derivativeTerm,[theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 L1 L2 L3],[theta1_num theta2_num 0 0 0 0 .5 .5 .1]));
    %uncomment to see single line equation% theta_t_linear = simplify(theta_t_constantTerm + theta_t_derivativeTerm*[q; dq]);
    %theta_t_linear = simplify(subs(theta_t_linear,[L1 L2 L3],[.5 .5 .1]))

    %First order approximation for zR
    zR_constantTerm = simplify(subs(zR,[theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 L1 L2 L3],[theta1_num theta2_num 0 0 0 0 .5 .5 .1]));
    zR_derivativeTerm = jacobian(zR,[q; dq]);
    zR_derivativeTerm = simplify(subs(zR_derivativeTerm,[theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 L1 L2 L3],[theta1_num theta2_num 0 0 0 0 .5 .5 .1]));
    %uncomment to see single line equation% zR_linear = simplify(zR_constantTerm + zR_derivativeTerm*[q; dq]);
    %zR_linear = simplify(subs(zR_linear,[L1 L2 L3],[.5 .5 .1]))

    %First order approximation for dtheta_t
    dtheta_t_constantTerm = simplify(subs(dtheta_t,[theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 L1 L2 L3],[theta1_num theta2_num 0 0 0 0 .5 .5 .1]));
    dtheta_t_derivativeTerm = jacobian(dtheta_t,[q; dq]);
    dtheta_t_derivativeTerm = simplify(subs(dtheta_t_derivativeTerm,[theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 L1 L2 L3],[theta1_num theta2_num 0 0 0 0 .5 .5 .1]));


    %First order approximation for dzR
    dzR_constantTerm = simplify(subs(dzR,[theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 L1 L2 L3],[theta1_num theta2_num 0 0 0 0 .5 .5 .1]));
    dzR_derivativeTerm = jacobian(dzR,[q; dq]);
    dzR_derivativeTerm = simplify(subs(dzR_derivativeTerm,[theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 L1 L2 L3],[theta1_num theta2_num 0 0 0 0 .5 .5 .1]));

    %Following the following convention of the 'x' matrix in y = Tx + b: [xW theta1 theta2 theta3 dxW dtheta1 dtheta2 dtheta3]'
    xW_row = [1 0 0 0 0 0 0 0];
    phi_row = [0 1 -1 1 0 0 0 0]; % Note phi = theta1 - theta2 + theta3
    dxW_row = [0 0 0 0 1 0 0 0];
    dphi_row = [0 0 0 0 0 1 -1 1];

    %Placing the Transformation matrix 'T' together
    T_mat = [xW_row; theta_t_derivativeTerm; zR_derivativeTerm; phi_row; dxW_row; dtheta_t_derivativeTerm; dzR_derivativeTerm; dphi_row];
    T_mat = vpa(simplify(T_mat));
    %Placing the constant 'b' matrix together in y = Tx + b
    T_constTerms_mat = [0; theta_t_constantTerm; zR_constantTerm; pi/2; 0; dtheta_t_constantTerm; dzR_constantTerm; 0];
    T_constTerms_mat = vpa(simplify(T_constTerms_mat));

    T = T_mat;
    V = T_constTerms_mat;


end