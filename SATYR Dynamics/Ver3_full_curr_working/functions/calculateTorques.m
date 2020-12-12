function [tao,tao_desired] = calculateTorques(X,p)

global enableSaturation K

theta1_num = p.theta1_num;
theta2_num = p.theta2_num;
xW = X(:,1);
dtheta1 = X(:,6); 
dtheta2 = X(:,7);
dtheta3 = X(:,8);

dtheta = {dtheta1,dtheta2,dtheta3};

tao = zeros(length(xW),3);
tao_desired = zeros(length(xW),3);
for k = 1:length(xW)
    X_ref = [0 theta1_num theta2_num 0 0 0 0 0]'; 
    tau_ref_10 = [0, -5.13599, -2.74328]';
    
    this_X = X(k,:)';
    this_X = this_X - X_ref;
    torque = -K*this_X;
    %Ideal torque we need
    tao_desired(k,:) = (torque + tau_ref_10)';
 
    for i = 1:3
        if enableSaturation == "linear"    
            % Torque-speed saturation (linear)
            % max_stall_torque = 2.83 | no_load_speed = 209.4 rad/s | Gear ratio = 6
            tau_bg_max = 2.8 - (2.8/209)*abs(dtheta{i}(k)); %Fitting the curve and then flipping the sign
            if(tau_bg_max < 0)
                tau_bg_max = 0;
            end
            tau_bg_min = -tau_bg_max;
            tau_g_max = 6*tau_bg_max;
            tau_g_min = 6*tau_bg_min;
            if torque(i) > tau_g_max
                torque(i) = tau_g_max;  
            elseif torque(i) < tau_g_min
                torque(i) = tau_g_min;
            end
        elseif enableSaturation == "cutoff"
            if torque(i) > 17
                torque(i) = 17;  
            elseif torque(i) < -17
                torque(i) = -17;
            end
        end
    end
    tau = torque + tau_ref_10;
    tao(k,:) = tau';
end

end