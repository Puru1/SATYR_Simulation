function p = getParams()

%Gravity
p.g = 9.81;

%Link masses
valM.mW = .42972;
valM.cm1 = .66162;
valM.cm2 = .9061;
valM.mR = 15.513;
p.valM = valM;

%Link lengths
valL.L1 = .180;
valL.L2 = .100;
valL.L3 = .118;
p.valL = valL;

%Linearization angle
p.use_joints = false;
p.theta1_num = pi/18;%angle of linearization
p.theta2_num = findTheta2(p.theta1_num, p.use_joints, valM, valL);
p.theta3_num = 0;

%CAD values
p.mK = .04; %currently not used
p.mH = .4;  %currently not used
p.R = 0.05; %radius of wheel


%Inertial values (from CAD model)
IW = [[.000423   0     0    ];
      [   0   .00075   0    ];
      [   0      0  .000423 ]];
  
ICM1 = [[.0127   0        0     ];
       [   0   .0125   -.002    ];
       [   0      0   .000777   ]];
   
ICM2 = [[.00527   0        0      ];
       [   0   .00482   .00011    ];
       [   0   .00011   .00111    ]];
   
IR = [[.23368    0     0    ];
      [   0   .19732   0    ];
      [   0      0   .06132 ]];
  
p.IW = IW;
p.IK = ICM1;
p.IH = ICM2;
p.IR = IR; 

% Controller params;
p.Qq = eye(8);
p.Ru = eye(3);

end