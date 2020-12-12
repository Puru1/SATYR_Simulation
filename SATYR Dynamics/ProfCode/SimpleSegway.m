function dX = SimpleSegway(t,X)

global g mW mR IW IR h R K
xW = X(1); 
theta = X(2); 
dxW = X(3); 
dtheta = X(4); 
tau = 0;

H = [[ mR + mW + IW/R^2 h*mR*cos(theta)];
    [h*mR*cos(theta) IR + (mR*(2*h^2*cos(theta)^2 + 2*h^2*sin(theta)^2))/2]];
C = [-dtheta^2*h*mR*sin(theta);
        -g*h*mR*sin(theta)];
tau = -K*X;
u = -0.5*[dxW; dtheta] + tau*[1/R; -1];
ddq = H\(u - C);
 
dX = [[dxW; dtheta]; [ddq(1); ddq(2)]];

end