function dX = SimpleSegway(t,X)

global K 
%Parameters
g = 9.81;
mw = .2;
mm = 0;
m1 = .15;
m2 = .15;
mr = 15;

Rw = .1;
Rm = .04;
L1 = .25;
L2 = .25;
L3 = .08;

% symM = [mw, mm, m1, m2, mr]; 
% valM = [0.2000 0.6000 0.1500 0.1500 15.0000];
% subs(symM, valM);
% 
% symL = [Rw, Rm, L1, L2, L3];
% valL =[0.1000    0.0400    0.2500    0.2500    0.0800];
% subs(symL, valL);

%Variable substitution
X1 = X(1); 
T1 = X(2); 
t2 = X(3);
t3 = X(4);
X1dot = X(5); 
T1dot = X(6); 
t2dot = X(7);
t3dot = X(8);

t %Time elapsed counter

%X_ref = [0, pi/6, -(5*pi)/12, 1.5421953896773856832653941961984, 0, 0, 0, 0]';
X_ref = [0, pi/18, -0.3079, 0, 0, 0, 0, 0]';


H =[[                                                                                                                                     m1 + m2 + 3*mm + mr + (3*mw)/2,                                                          L3*mr*cos(T1 + t2 + t3) + (L2*m2*cos(T1 + t2))/2 + 2*L2*mm*cos(T1 + t2) + L2*mr*cos(T1 + t2) + (L1*m1*cos(T1))/2 + L1*m2*cos(T1) + 2*L1*mm*cos(T1) + L1*mr*cos(T1),                                                                            L3*mr*cos(T1 + t2 + t3) + (L2*m2*cos(T1 + t2))/2 + 2*L2*mm*cos(T1 + t2) + L2*mr*cos(T1 + t2),                                  L3*mr*cos(T1 + t2 + t3)]
[ L3*mr*cos(T1 + t2 + t3) + (L2*m2*cos(T1 + t2))/2 + 2*L2*mm*cos(T1 + t2) + L2*mr*cos(T1 + t2) + (L1*m1*cos(T1))/2 + L1*m2*cos(T1) + 2*L1*mm*cos(T1) + L1*mr*cos(T1), (L1^2*m1)/3 + L1^2*m2 + (L2^2*m2)/3 + 2*L1^2*mm + 2*L2^2*mm + L1^2*mr + L2^2*mr + (4*L3^2*mr)/3 + (3*Rm^2*mm)/2 + 2*L1*L3*mr*cos(t2 + t3) + L1*L2*m2*cos(t2) + 4*L1*L2*mm*cos(t2) + 2*L1*L2*mr*cos(t2) + 2*L2*L3*mr*cos(t3), (L2^2*m2)/3 + 2*L2^2*mm + L2^2*mr + (4*L3^2*mr)/3 + Rm^2*mm + L1*L3*mr*cos(t2 + t3) + (L1*L2*m2*cos(t2))/2 + 2*L1*L2*mm*cos(t2) + L1*L2*mr*cos(t2) + 2*L2*L3*mr*cos(t3), (4*L3^2*mr)/3 + L1*L3*mr*cos(t2 + t3) + L2*L3*mr*cos(t3)]
[                                                                       L3*mr*cos(T1 + t2 + t3) + (L2*m2*cos(T1 + t2))/2 + 2*L2*mm*cos(T1 + t2) + L2*mr*cos(T1 + t2),                                                     (L2^2*m2)/3 + 2*L2^2*mm + L2^2*mr + (4*L3^2*mr)/3 + Rm^2*mm + L1*L3*mr*cos(t2 + t3) + (L1*L2*m2*cos(t2))/2 + 2*L1*L2*mm*cos(t2) + L1*L2*mr*cos(t2) + 2*L2*L3*mr*cos(t3),                                                                                        (L2^2*m2)/3 + 2*L2^2*mm + L2^2*mr + (4*L3^2*mr)/3 + Rm^2*mm + 2*L2*L3*mr*cos(t3),                         (4*mr*L3^2)/3 + L2*mr*cos(t3)*L3]
[                                                                                                                                            L3*mr*cos(T1 + t2 + t3),                                                                                                                                                                         (L3*mr*(4*L3 + 3*L1*cos(t2 + t3) + 3*L2*cos(t3)))/3,                                                                                                                                         (L3*mr*(4*L3 + 3*L2*cos(t3)))/3,                                            (4*L3^2*mr)/3]];

[eigVec, eigVal] = eig(H);
eigVal_ar = diag(eigVal);
cond = max(eigVal_ar)/min(eigVal_ar);


C =[                        - (L2*T1dot^2*m2*sin(T1 + t2))/2 - 2*L2*T1dot^2*mm*sin(T1 + t2) - L2*T1dot^2*mr*sin(T1 + t2) - (L2*m2*t2dot^2*sin(T1 + t2))/2 - 2*L2*mm*t2dot^2*sin(T1 + t2) - L2*mr*t2dot^2*sin(T1 + t2) - (L1*T1dot^2*m1*sin(T1))/2 - L1*T1dot^2*m2*sin(T1) - 2*L1*T1dot^2*mm*sin(T1) - L1*T1dot^2*mr*sin(T1) - L3*T1dot^2*mr*sin(T1 + t2 + t3) - L3*mr*t2dot^2*sin(T1 + t2 + t3) - L3*mr*t3dot^2*sin(T1 + t2 + t3) - 2*L3*mr*t2dot*t3dot*sin(T1 + t2 + t3) - L2*T1dot*m2*t2dot*sin(T1 + t2) - 4*L2*T1dot*mm*t2dot*sin(T1 + t2) - 2*L2*T1dot*mr*t2dot*sin(T1 + t2) - 2*L3*T1dot*mr*t2dot*sin(T1 + t2 + t3) - 2*L3*T1dot*mr*t3dot*sin(T1 + t2 + t3)
 - (L2*g*m2*sin(T1 + t2))/2 - 2*L2*g*mm*sin(T1 + t2) - L2*g*mr*sin(T1 + t2) - (L1*g*m1*sin(T1))/2 - L1*g*m2*sin(T1) - 2*L1*g*mm*sin(T1) - L1*g*mr*sin(T1) - L3*g*mr*sin(T1 + t2 + t3) - (L1*L2*m2*t2dot^2*sin(t2))/2 - 2*L1*L2*mm*t2dot^2*sin(t2) - L1*L2*mr*t2dot^2*sin(t2) - L2*L3*mr*t3dot^2*sin(t3) - L1*L3*mr*t2dot^2*sin(t2 + t3) - L1*L3*mr*t3dot^2*sin(t2 + t3) - 2*L1*L3*mr*t2dot*t3dot*sin(t2 + t3) - L1*L2*T1dot*m2*t2dot*sin(t2) - 4*L1*L2*T1dot*mm*t2dot*sin(t2) - 2*L1*L2*T1dot*mr*t2dot*sin(t2) - 2*L2*L3*T1dot*mr*t3dot*sin(t3) - 2*L2*L3*mr*t2dot*t3dot*sin(t3) - 2*L1*L3*T1dot*mr*t2dot*sin(t2 + t3) - 2*L1*L3*T1dot*mr*t3dot*sin(t2 + t3)
                                                                                                                                                                                                                                                                                                                                    L1*L3*T1dot^2*mr*sin(t2 + t3) - 2*L2*g*mm*sin(T1 + t2) - L2*g*mr*sin(T1 + t2) - L3*g*mr*sin(T1 + t2 + t3) - L2*L3*mr*t3dot^2*sin(t3) - (L2*g*m2*sin(T1 + t2))/2 + (L1*L2*T1dot^2*m2*sin(t2))/2 + 2*L1*L2*T1dot^2*mm*sin(t2) + L1*L2*T1dot^2*mr*sin(t2) - 2*L2*L3*T1dot*mr*t3dot*sin(t3) - 2*L2*L3*mr*t2dot*t3dot*sin(t3)
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  L3*mr*(L1*T1dot^2*sin(t2 + t3) - g*sin(T1 + t2 + t3) + L2*T1dot^2*sin(t3) + L2*t2dot^2*sin(t3) + 2*L2*T1dot*t2dot*sin(t3))];
X = X - X_ref;
tau = -K*X;
tau = [tau(1)/Rw; -tau(1); -tau(2); -tau(3)];
u = 0*[X1dot; T1dot; t2dot; t3dot] + tau;
ddq = H\(u - C);

dX = [[X1dot; T1dot; t2dot; t3dot]; [ddq(1); ddq(2); ddq(3); ddq(4)]];

end