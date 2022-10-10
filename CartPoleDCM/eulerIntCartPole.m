function [T,X,q_ref] = eulerIntCartPole(p, X_ref, simTime,q0,K)

    dt = .001; % 1kHz
    q_state =  q0;
    q_DCM = zeros(4,1);
    dx_des_prev = 0;
    x_des_prev = 0;
    T = [];
    X = [];
    q_ref = [];
 
    %Robot Parameters
    g = p.g;
    h_R=p.L1; %Robot CoM height
    M_R=p.mCM; %Robot mass (kg)
    m_R=p.mW; %Cart mass (kg)
    omega_R=sqrt(g/h_R); %Robot natural freq.
    
    mCM = p.mCM;
    mW = p.mW;
    L1 = p.L1;
    R = p.R;
    
    for time = 0:dt:simTime
        time
        x_R = q_state(1);
        th_R = q_state(2);
        dx_R = q_state(3);
        dth_R = q_state(4);

        theta1 = th_R;
        dtheta1 = dth_R;
        
        %DCM state transform
        q_DCM(1) = x_R;
        q_DCM(2) = dx_R;
        q_DCM(3) = x_R + h_R*th_R; %CoM
        q_DCM(4) = th_R + (dth_R/omega_R); %unitless DCM
        
        %Reference vector
%         ddx_des = 0;
%         dx_des = dx_des_prev + ddx_des*dt;
%         x_des = x_des_prev + dx_des*dt;
%         th_R_des = -time^2 + 3*time;
%         dth_R_des = -2*time;
%         DCM_R_des = th_R_des + dth_R_des/omega_R;
%         X_ref = [x_des;dx_des;0;DCM_R_des];
%         
%         dx_des_prev = dx_des;
%         x_des_prev = x_des;

        %Testing LQR feedback controller
        q_error = q_DCM - X_ref;
        tau1 =  -(K(1)*q_error(1) + K(2)*q_error(2) + K(3)*q_error(3) + K(4)*q_error(4));
        
        
        %%Using the nonlinear dynamics with simplest IVP (initial value problem) using Euler integration
        
        %Solve for ddx_R (WIP)
        %ddx_R = (h_R*tau1 + R*tau1*cos(th_R) + h_R*h_R*R*dth_R*dth_R*M_R*sin(th_R) - h_R*R*g*M_R*cos(th_R)*sin(th_R))/(h_R*R*(M_R + m_R - M_R*cos(th_R)*cos(th_R)));
        
        %Solve for ddx_R (Cart-Pole)
        ddx_R = (L1*mCM*sin(theta1)*dtheta1^2 + tau1 - g*mCM*cos(theta1)*sin(theta1))/(- mCM*cos(theta1)^2 + mCM + mW);

        %Integrate & set old vals equal to new
        q_state(1) = x_R + dt*(dx_R); % Current time step val of dx_R to find current x_R
        q_state(3) = dx_R + dt*(ddx_R); % Current time step acceleration to find next vel

        %Solve for ddth_R (WIP)
        %ddth_R = -(R*M_R*tau1 + R*m_R*tau1 + h_R*M_R*tau1*cos(th_R) - h_R*R*g*M_R*M_R*sin(th_R) - h_R*R*g*M_R*...
        %         m_R*sin(th_R) + h_R*h_R*R*dth_R*dth_R*M_R*M_R*cos(th_R)*sin(th_R))/(h_R*h_R*R*M_R*(M_R + m_R - M_R*cos(th_R)*cos(th_R)));
       
        %Solve for ddth_R (Cart-Pole)
        ddth_R = -(L1*mCM*cos(theta1)*sin(theta1)*dtheta1^2 + tau1*cos(theta1) - g*mCM*sin(theta1) - g*mW*sin(theta1))/(L1*(- mCM*cos(theta1)^2 + mCM + mW));
      
        %Integrate & set old vals equal to new
        q_state(2) = th_R + dt*(dth_R);
        q_state(4) = dth_R + dt*(ddth_R);
        
        T =[T;time];
        X =[X;q_state'];
        q_ref = [q_ref;X_ref'];
    end 
    
end 
