function [tau_max,tau_min,omega_out,actuatorFlag] = checkTorque(dq,tau_des)

    dtheta = dq(2); % ignoring dxW
    Nw = 6;

    % Setting gearing ratios depending on joint 

    % Bound operating speed (if needed)
    omega_br = Nw*dtheta; %before reduction
    %omega_br = dtheta(i);
    if(omega_br > 209.4)
        omega_br = 209.4;
        omega_out = 209.4/Nw;
        %omega_out(i) = 209.4;
    elseif(omega_br < -209.4)
        omega_br = -209.4;
        omega_out = -209.4/Nw;
        %omega_out(i) = -209.4;
    else
        omega_out = omega_br/Nw;
        %omega_out(i) = omega_br;
    end

    %Adding torque saturation
    % max_stall_torque = 2.83 | no_load_speed = 209.4 rad/s | Gear ratio = 6
    tau_bg_max = 2.8 - (2.8/209.4)*abs(omega_br); %Fitting the curve 
    if(tau_bg_max > 2.8)
        tau_bg_max = 2.8;
    end 

    tau_bg_min = -tau_bg_max;
    tau_g_max = Nw*tau_bg_max;
    tau_g_min = Nw*tau_bg_min;

    tau_max = tau_g_max;
    tau_min = tau_g_min;

    if (tau_des > tau_g_max)
        actuatorFlag = false;
    elseif (tau_des < tau_g_min)
        actuatorFlag = false;
    else
        actuatorFlag = true;
    end


end