function m_list = import_m_list()

m_list.q = {
    'xW' 'q(1)';
    'theta1' 'q(2)';
    };

m_list.dq = {
    'dxW' 'dq(1)';
    'dtheta1' 'dq(2)';
    };

m_list.L = {
    'L1' 'L(1)';
    'R'  'L(2)';
    };

m_list.M = {
    'mW' 'vMass(1)';
    'mCM' 'vMass(2)';
    };

m_list.p = {
    'g' 'g';
};
end
