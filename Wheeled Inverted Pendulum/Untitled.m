%% VARIABLE DECLARATION
clc;
clear all;
addpath fnc

syms xR thetaR xP zP real
syms dxR dthetaR dxP dxP real
syms ddxR ddthetaR ddxP ddzP tau1 real
syms g M mCM IW ICM mR L R real

%States
q = [xR; thetaR];
dq = [dxR; dthetaR];
ddq = [ddxR; ddthetaR];


xP = L*sin(thetaR);
dxP = L*cos(thetaR)*dthetaR + dxR;

zP = L*cos(thetaR);
dzP = -L*sin(thetaR)*dthetaR;

T = (1/2)*M*dxR^2 + (1/2)*mR*(dxP^2 + dzP^2);

V = mR*g*L*cos(thetaR);

L = T - V;
dLdqi = transpose(jacobian(L,q));
dLddqi = transpose(jacobian(L,dq));
eqs = simplify(jacobian(dLddqi,q)*dq + jacobian(dLddqi,dq)*ddq - dLdqi);