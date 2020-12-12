function grfFlag = checkGRF(p,q,dq,tau_des)

%% PARAMETERS & INITIALIZATONS
    g = p.g;
    mW = p.valM.mW;
    mCM1 = p.valM.cm1;
    mCM2 = p.valM.cm2;
    mR = p.valM.mR;
    L1 = p.valL.L1; 
    L2 = p.valL.L2;
    L3 = p.valL.L3;
    L = [L1 L2 L3];
    Mass = [mW,mCM1,mCM2,mR];

    % Jacobian and adjusting matrices
    JvR = fnc_JvR(q,L); 
    %These next to steps subs xW = theta * R and recreeate the matrix
    JvR_reduced = JvR(:,2:4); 
    JvR_reduced(1,1) = JvR_reduced(1,1) + p.R;
    JvR_inv = pinv(-JvR_reduced');    

    % External forces calc
    %Acmm = fnc_Acmm(q,dq,Mass,L);
    %dAcmm = fnc_dAcmm(q,dq,Mass,L);

    % Acmm(q) * ddq + dAcmm * dq = [pl;pr]
    % Fext = Acmm*ddq + dAcmm*dq - [0;0;(mR*g)]; %Cant do because we dont
    % have access to ddq
    % GRFz = Fext(3);
    GRF_anticipated = JvR_inv*tau_des;
    if(GRF_anticipated < 0)
        grfFlag = false;
    else
        grfFlag = true;
    end
end

