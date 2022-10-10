function grfFlag = checkGRF(p,q,dq,tau_des)

    % Jacobian
    JvCM = fnc_JvCM(q,p.L1);
    JvCM_reduced = [JvCM(1,:);JvCM(3,:)];
    JvCM_inv = inv(-JvCM_reduced');

    GRF_anticipated = JvCM_inv*tau_des;
    if(GRF_anticipated(2,2) < 0)
        grfFlag = false;
    else
        grfFlag = true;
    end
end

