function m_list = import_m_list()

m_list.q = {
    'xW' 'q(1)';
    'theta1' 'q(2)';
    'theta2' 'q(3)';
    'theta3' 'q(4)';
    };

m_list.dq = {
    'dxW' 'dq(1)';
    'dtheta1' 'dq(2)';
    'dtheta2' 'dq(3)';
    'dtheta3' 'dq(4)';
    };

m_list.L = {
    'L1' 'L(1)';
    'L2' 'L(2)';
    'L3' 'L(3)';
    'R'  'L(4)';
    };

m_list.M = {
    'mW' 'vMass(1)';
    'mCM1' 'vMass(2)';
    'mCM2' 'vMass(3)';
    'mR' 'vMass(4)';
    };

m_list.p = {
    'g' 'g';
};
end
