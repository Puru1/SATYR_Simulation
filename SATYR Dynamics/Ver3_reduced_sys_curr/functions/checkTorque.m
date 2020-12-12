function [tau_max,tau_min,omega_out, actuatorFlag] = checkTorque(dq,tau_des)

dtheta = dq(2:4) % ignoring dxW
Nw = 6;
Nk = 6;
Nh = 6;
tau_max = zeros(3,1);
tau_min = zeros(3,1);
omega_out = zeros(3,1);
actuatorFlag = [true;true;true];

% Torque-speed saturation (linear)
for i = 1:3
    
    % Setting gearing ratios depending on joint
    if i == 1
        N = Nw;
    elseif i == 2
        N = Nk;
    else
        N = Nh;
    end 

    % Bound operating speed (if needed)
    omega_br = N*dtheta(i); %before reduction
    %omega_br = dtheta(i);
    if(omega_br > 209.4)
        omega_br = 209.4;
        omega_out(i) = 209.4/N;
        %omega_out(i) = 209.4;
    elseif(omega_br < -209.4)
        omega_br = -209.4;
        omega_out(i) = -209.4/N;
        %omega_out(i) = -209.4;
    else
        omega_out(i) = omega_br/N;
        %omega_out(i) = omega_br;
    end
    
    %Adding torque saturation
    % max_stall_torque = 2.83 | no_load_speed = 209.4 rad/s | Gear ratio = 6
    tau_bg_max = 2.8 - (2.8/209.4)*abs(omega_br); %Fitting the curve 
    if(tau_bg_max > 2.8)
        tau_bg_max = 2.8;
    end 
    
    tau_bg_min = -tau_bg_max;
    tau_g_max = N*tau_bg_max;
    tau_g_min = N*tau_bg_min;

    tau_max(i) = tau_g_max;
    tau_min(i) = tau_g_min;
    
    if (tau_des(i) > tau_g_max)
        actuatorFlag(i) = false;
    elseif (tau_des(i) < tau_g_min)
        actuatorFlag(i) = false;
    else
        actuatorFlag(i) = true;
    end

end

end