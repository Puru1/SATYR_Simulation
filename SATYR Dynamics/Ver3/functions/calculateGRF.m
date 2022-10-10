function GRFz = calculateGRF(X,p,tao)
    % Acmm(q) * ddq + dAcmm * dq = [pl;pr]
    % Fext = Acmm*ddq + dAcmm*dq - [0;0;(mR*g)]; %Cant do because we dont
    ddq = zeros(height(X),4);
    GRF = zeros(height(X),3);
    for i = 1: height(X)
        % Variable organization
        mR = sum(p.M);
        q = X(i,1:4);
        dq = X(i,5:8);
        u = [-tao(i,1)/p.R; tao(i,1); tao(i,2); tao(i,3)];
        H = fnc_H(q,p.L,p.M);
        C = fnc_C(q,dq,p.L,p.M,p.g);
        Acmm = fnc_Acmm(q,p.M,p.L);
        dAcmm = fnc_dAcmm(q,dq,p.M,p.L);
        
        % Calculating CoM & radius from contact point to CoM
        CoM = [p.M(1);0;0]*q(1) + p.M(2)*fnc_PosCM1(q,p.L) + p.M(3)*fnc_PosCM2(q,p.L) + p.M(4)*fnc_PosCoM_R(q,p.L);
        CoM = CoM/sum(p.M);
        r = CoM - [q(1);0;0];
        Sext = [1 0 0; 0 0 1; r(3) 0 r(1)];
        
        % External forces
        H_inv = inv(H);
        ddq(i,:) = H_inv *(u-C);
        Fext = Acmm*ddq(i,:)' + dAcmm*dq' - [0;0;mR*p.g];
        GRF(i,:) = Sext*Fext;
    end
    GRFz = GRF(:,2);
end