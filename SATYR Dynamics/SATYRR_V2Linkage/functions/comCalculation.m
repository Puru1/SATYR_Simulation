function theta_out = comCalculation(theta_in)
    
    %We make the assumption that we are given a fixed theta3 and theta1 and
    %are searching for theta2
    theta1_num = theta_in(1);
    theta3_num = theta_in(3);
    theta2_num = 0;
    syms xW theta1 theta2 theta3
    syms dxW dtheta1 dtheta2 dtheta3
    syms ddxW ddtheta ddtheta2 ddtheta3 tau1 tau2 tau3
    syms g mW mR IW IR L1 L2 L3 R m_joint1 m_joint2 m_motor

    %global theta2_num_rad theta1_num

    %States
    q = [xW; theta1; theta2; theta3];
    dq = [dxW; dtheta1; dtheta2; dtheta3];
    ddq = [ddxW; ddtheta; ddtheta2; ddtheta3];

    %Positions
    x_joint1 = L1*sin(theta1);
    x_joint2 = L1*sin(theta1) + L2*sin(theta1+theta2);
    xR = L1*sin(theta1) + L2*sin(theta1+theta2) + L3*sin(theta1+theta2+theta3);

    z_joint1 = L1*cos(theta1);
    z_joint2 = L1*cos(theta1) + L2*cos(theta1+theta2);
    zR = L1*cos(theta1) + L2*cos(theta1+theta2) + L3*cos(theta1+theta2+theta3);

    xcom = ((m_joint1*x_joint1) + (m_joint2*x_joint2) + (mR*xR))/(m_joint1 + m_joint2 + mR);
    ycom = 0;
    zcom = ((m_joint1*z_joint1) + (m_joint2*z_joint2) + (mR*zR))/(m_joint1 + m_joint2 + mR);

    com = [xcom;ycom;zcom];

    %% Point of linearization
    %We want our xcom to equal zero. We assume here that theta1 is given and
    %theta3 is 0. 

    alpha = m_joint2*L2 + mR*L2 + mR*L3;
    gamma = -m_joint1*L1*sin(theta1) - m_joint2*L1*sin(theta1) - mR*L1*sin(theta1);
    theta2_num_analy = asin(gamma/alpha) - theta1;

    angle_array = zeros(100,2);
    index  = 0;
    for comMass = .1:.1:10
        index = index +1;
        theta2_num = double(simplify(subs(theta2_num_analy,[theta1 theta3 L1 L2 L3 m_joint1 m_joint2 mR],[theta1_num 0 .5 .5 .1 .04 .4 comMass])));
        theta2_num = theta2_num*(180/pi); %in degrees
        angle_array(index,1) = comMass;
        angle_array(index, 2) = theta2_num;
        %Verify that xcom is indeed zero
        %xcom_num = double(simplify(subs(xcom,[theta1 theta2 theta3 L1 L2 L3 m_joint1 m_joint2 mR],[theta1_num theta2_num_rad 0 .5 .5 .1 .04 .4 comMass])));
    end 


    theta2_num_rad = double(simplify(subs(theta2_num_analy,[theta1 theta3 L1 L2 L3 m_joint1 m_joint2 mR],[theta1_num 0 .5 .5 .1 .04 .4 15])))
    xcom_verified = double(simplify(subs(xcom,[theta1 theta2 theta3 L1 L2 L3 m_joint1 m_joint2 mR],[theta1_num theta2_num_rad 0 .5 .5 .1 .04 .4 15])))

    %%
    figure(1);
    plot(angle_array(:,1),angle_array(:,2));
    title('Stabilizing \theta_{2} angle vs mR');
    xlabel('Robot COM Weight (kg)');
    ylabel('Angle (degrees)');
    %%
    %Fext_x = ddtheta2*(mR*(L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3)) + (L2*m_joint2*cos(theta1 + theta2))/2) - dtheta2*(mR*(L3*sin(theta1 + theta2 + theta3)*(dtheta1 + dtheta2 + dtheta3) + L2*sin(theta1 + theta2)*(dtheta1 + dtheta2)) + (L2*m_joint2*sin(theta1 + theta2)*(dtheta1 + dtheta2))/2) - dtheta1*(mR*(L3*sin(theta1 + theta2 + theta3)*(dtheta1 + dtheta2 + dtheta3) + L2*sin(theta1 + theta2)*(dtheta1 + dtheta2) + L1*dtheta1*sin(theta1)) + m_joint2*(L2*sin(theta1 + theta2)*(dtheta1 + dtheta2) + L1*dtheta1*sin(theta1)) + (L1*dtheta1*m_joint1*sin(theta1))/2) + ddxW*(mR + mW + m_joint1 + m_joint2) + ddtheta1*(m_joint2*((L2*cos(theta1 + theta2))/2 + L1*cos(theta1)) + mR*(L2*cos(theta1 + theta2) + L1*cos(theta1) + L3*cos(theta1 + theta2 + theta3)) + (L1*m_joint1*cos(theta1))/2) + L3*ddtheta3*mR*cos(theta1 + theta2 + theta3) - L3*dtheta3*mR*sin(theta1 + theta2 + theta3)*(dtheta1 + dtheta2 + dtheta3);
    %Fext_x = double(simplify(subs(Fext_x,[xW theta1 theta2 theta3 dxW dtheta1 dtheta2 dtheta3 ddxW ddtheta1 ddtheta2 ddtheta3 L1 L2 L3 m_joint1 m_joint2 mR],[0 theta1_num theta2_num_rad 0 0 0 0 0 0 0 0 0 .5 .5 .1 .1 .1 15])))

