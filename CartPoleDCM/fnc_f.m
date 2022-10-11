function [f] = fnc_f(q,p)

f = zeros(2,1);

  f(1,1)=(tau1 + h_R*dth_R^2*M_R*sin(th_R) - g*M_R*cos(th_R)*sin(th_R))/(M_R + m_R - M_R*cos(th_R)^2);
  f(2,1)=-(tau1*cos(th_R) - g*M_R*sin(th_R) - g*m_R*sin(th_R) + h_R*dth_R^2*M_R*cos(th_R)*...
         sin(th_R))/(h_R*(M_R + m_R - M_R*cos(th_R)^2));

 