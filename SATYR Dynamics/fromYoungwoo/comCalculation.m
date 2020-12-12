function theta2_out = comCalculation(theta_in, use_joints)
    global valM valL
    %Params
    m_joint2 = 2*valM(2);
    m_link1 = valM(3);
    m_link2 = valM(4);
    mR = valM(5);
    L1 = valL(3);
    L2 = valL(4);
    L3 = valL(5);
    theta1_num = theta_in(1);
    %We make the assumption that we are given a fixed theta1_num and theta3_num and
    %are searching for theta2   
    
    if use_joints == false
        gamma = -(1/2)*m_link1*L1*sin(theta1_num) - m_link2*L1*sin(theta1_num) - mR*L1*sin(theta1_num);
        alpha = (1/2)*m_link2*L2 + mR*L2 + mR*L3;
        theta2_num = asin(gamma/alpha) - theta1_num   
    else
        gamma = -(1/2)*m_link1*L1*sin(theta1_num) - m_link2*L1*sin(theta1_num) -m_joint2*L1*sin(theta1_num) - mR*L1*sin(theta1_num);
        alpha = (1/2)*m_link2*L2 + m_joint2*L2 + mR*L2 + mR*L3;
        theta2_num = asin(gamma/alpha) - theta1_num
    end


theta2_out = theta2_num;
    