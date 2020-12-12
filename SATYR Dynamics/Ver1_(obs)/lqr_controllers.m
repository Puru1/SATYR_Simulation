%%
%LQR Infinite-Horizon 
A  = ?
B = ?
C = ?
D = ?
CT = ss(A,B,C,D);
DT = c2d(CT,T)
phi = DT.A;
gamma = DT.B;
Q1 = eye(10);
Q2 = eye(3);
K = dlqr(phi*alpha,gamma*alpha,Q1,Q2)
%cl_p = eig(phi-gamma*K) %closed loop poles
x = zeros(10,51);
x0 = zeros(10,1);
x(:,1) = x0;
y = zeros(3,50);
for i = [1:51]
    x(:,i+1) = (phi-gamma*K)*x(:,i);
    y(:,i) = C*x(:,i);
end
u = -K*x;
steps = [0:0.2:10.2];

%%
%Finite-Horizon LQR Controller
S = cell(1,52);
S{52} = Q1;
K = cell(1,52);
K{52} = zeros(10,1);
M = cell(1,52);
for i = [0:50]  %pg 367
    k = 50-i;   %correct k, used to find new phi
    ki = k+2;   %k for indexing
    phi_new{ki} = 1.1^k*phi;    %new phi
    phi_new{ki-1} = 1.1^(k-1)*phi; 
    M{ki} = S{ki}-S{ki}*gamma*(Q2+gamma'*S{ki}*gamma)^-1*gamma'*S{ki};
    K{ki-1} = (Q2+gamma'*S{ki}*gamma)^-1*gamma'*S{ki}*phi_new{ki-1};
    S{ki-1} = phi_new{ki}'*M{ki}*phi_new{ki}+Q1;
end

x = zeros(3,51);
x0 = [50,50,50]';
x(:,1) = x0;
y = zeros(3,50);
for i = [1:51]      %index x and y's
    x(:,i+1) = (phi_new{i+1}-gamma*K{i+1})*x(:,i);     
    y(:,i) = C*x(:,i);
end
u = zeros(2,51);
for i = [1:51]      %index u
    u(:,i) = -K{i+1}*x(:,i);  
    
end
steps = [0:0.2:10];