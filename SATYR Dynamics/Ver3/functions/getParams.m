function p = getParams()

%Gravity
p.g = 9.81;

%Link masses
valM.mW = .42972;
valM.cm1 = .66162;
valM.cm2 = .9061;
valM.mR = 5.63;
p.valM = valM;
p.M = [valM.mW, valM.cm1, valM.cm2, valM.mR];

%CAD values
p.mK = .04; %currently not used
p.mH = .4;  %currently not used
p.R = 0.06; %radius of wheel

%Link lengths
valL.L1 = .160;
valL.L2 = .200;
valL.L3 = .4;
p.valL = valL;
p.L = [valL.L1, valL.L2, valL.L3,p.R];

%Linearization angle intialization
p.use_joints = false;
p.theta1_num = 0;
p.theta2_num = 0;
p.theta3_num = 0;

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

%Features
p.enableSaturation = "cutoff";
p.captureVideoEnable = false;

end