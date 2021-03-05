function m_list = import_m_list()

m_list.q = {
    'xW' 'q(1)';
    'l_h' 'q(2)';
    'theta1' 'q(3)';
    };

m_list.dq = {
    'dxW' 'dq(1)';
    'dl_h' 'dq(2)';
    'dtheta1' 'dq(3)';
    };

m_list.L = {
    'L0' 'L(1)';
    'R'  'L(2)';
    };

m_list.M = {
    'mW' 'vMass(1)';
    'mB' 'vMass(2)';
    };

m_list.p = {
    'g' 'g';
    'Ks' 'Ks';
};
end
