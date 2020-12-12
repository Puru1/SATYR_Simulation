clc;
close all;
clear all;

syms xW theta 
syms dxW dtheta 
syms ddxW ddtheta tau
syms g mW mR IW IR h R

%States
q = [xW; theta];
dq = [dxW; dtheta];
ddq = [ddxW; ddtheta];

%Energy
xR = xW + h*sin(theta);
zR = h*cos(theta);
dxR = dxW + dtheta*h*cos(theta);
dzR = - dtheta*h*sin(theta);
Kin = mW*dxW^2/2 + IW*(dxW/R)^2/2 + mR*(dxR^2 + dzR^2)/2 + IR*dtheta^2/2;
Pot = mR*g*h*cos(theta);

%Lagrangian
L = Kin - Pot;
dLdqi = transpose(jacobian(L,q));
dLddqi = transpose(jacobian(L,dq));
eqs = simplify(jacobian(dLddqi,q)*dq + jacobian(dLddqi,dq)*ddq - dLdqi);

%Equations of motions H(q,dq)*ddq + C = tau
H = jacobian(dLddqi,dq)
C = simplify(jacobian(dLddqi,q)*dq - dLdqi)
u = [tau/R; -tau];

%Linearization: dX = A*X + B*tau
f = simplify(H\(u - C));
A = jacobian(f,[q; dq]);
A = simplify(subs(A,[xW theta dxW dtheta tau],[0 0 0 0 0]))
B = jacobian(f,tau);
B = simplify(subs(B,[xW theta dxW dtheta tau],[0 0 0 0 0]))










