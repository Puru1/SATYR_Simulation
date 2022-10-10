clear all
A = 1.0e+04 *[

         0         0         0         0    0.0001         0         0         0
         0         0         0         0         0    0.0001         0         0
         0         0         0         0         0         0    0.0001         0
         0         0         0         0         0         0         0    0.0001
         0   -0.0000    0.0000    0.0000         0         0         0         0
         0    0.6362   -4.6146   -2.7993         0         0         0         0
         0   -1.1620    8.4652    5.1198         0         0         0         0
         0   -0.0190   -0.0088    0.0840         0         0         0         0];
     
B = [    0         0         0
         0         0         0
         0         0         0
         0         0         0
    0.0110   -0.0010         0
  -78.4703  143.9675   -0.6466
  143.9875 -264.3123    1.6810
   -0.6466    1.6810   -2.9806];

C = eye(8);
D = 0;

T = .05;
CT = ss(A,B,C,D);
DT = c2d(CT,T)
phi = DT.A;
gamma = DT.B;
Q1 = diag([750 1 1 1 1 1 1 1]); %Q
Q2 = 1; %R
K = dlqr(phi,gamma,Q1,Q2)
cl_p = eig(phi-gamma*K);

endRange = 401;
x = zeros(8,endRange);
x0 = [0,pi/10, pi/10, pi/10, 0, 0, 0, 0]';
x(:,1) = x0;
y = zeros(8,(endRange-1));
for i = [1:endRange]
    x(:,i+1) = (phi-gamma*K)*x(:,i);
    y(:,i) = C*x(:,i);
end
u = -K*x;
steps = [0:0.05:(endRange*.05)];

%Graph
figure
stairs(steps,x(1,:))
title('SATYRR Discrete LQR xW Stabilization')
xlabel('Time (sec)')
ylabel('Distance (m)')
hold on
%stairs(steps,x(2,:))
%stairs(steps,x(3,:))
%stairs(steps,u(1,:))
%stairs(steps,u(2,:))
%stairs(steps,u(3,:))
legend('xW','u1','u2', 'u3')