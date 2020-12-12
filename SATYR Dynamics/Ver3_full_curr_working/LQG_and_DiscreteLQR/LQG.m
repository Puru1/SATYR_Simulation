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

%Designing Optimal State Feedback Gain
Q = diag([750 1 1 1 1 1 1 1]);
R = 1;
K= lqr(A,B,Q,R);

%Constructing the Kalman State Estimator
sys = ss(A,B,C,D);
Qn = eye(3);
Rn = eye(8);
Nn = 0;
[kest,L,P] = kalman(sys,Qn,Rn,Nn);

%Forming the LQG Regulator
regulator = lqgreg(kest, K);

%Reformulating matrices for simulation
Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];