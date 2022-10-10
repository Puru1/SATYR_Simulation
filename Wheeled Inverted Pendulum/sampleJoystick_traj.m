function X_ref =  sampleJoystick_traj(p,t)
    
    ddtheta_des = 0;
    max_tilt = .171/2;
    if(t <= 1)
        theta_des = max_tilt*t;
        ddx_des = p.L1*ddtheta_des + p.g*theta_des;
        dx_des = ddx_des *t;
        x_des = dx_des * t;
    elseif(t>1 && t<3)
        theta_des = (max_tilt*1) - max_tilt*(t-1);
        ddx_des =  (9.81*max_tilt) + p.g*theta_des;
        dx_des = ddx_des * t;
        x_des = dx_des * t;
    else
        theta_des = 0;
        dx_des = 0;
        x_des = 0;
    end 
    
    X_ref = [x_des, theta_des, dx_des, 0];
    % Solving dynamics for ddx

end