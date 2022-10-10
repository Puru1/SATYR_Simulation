function out = computeTorques(X,p,X_ref)

    n = length(X);
    tau_ref = fnc_tauLin();
    K = fnc_K();
%     K = [-70 -315 -40 -35]
    
    for i = 1: n
        %Calculate initial torques
        x = X(i,:)' - X_ref;
        tau_error = -K*x;

        % Ensure we normalize properly around linearization 
        tau_des = tau_error + tau_ref(2);

        tau = [tau_des/(p.R); -tau_des];
        u = tau';
        out(i,:) = u;
    end 
end 