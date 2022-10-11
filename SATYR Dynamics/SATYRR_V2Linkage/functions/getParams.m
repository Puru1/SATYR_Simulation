function p = getParams()

<<<<<<< HEAD
<<<<<<< HEAD
%Spring compression
p.deltaL = .03;

=======
>>>>>>> b6f1ecc8a3fb2eda4e78b6658502c98c386c0207
=======
%Spring compression
p.deltaL = .03;

>>>>>>> adding_folders
%Gravity
p.g = 9.81;

%Spring constant
<<<<<<< HEAD
<<<<<<< HEAD
p.Ks = 500;
=======
p.Ks = 1000;
>>>>>>> b6f1ecc8a3fb2eda4e78b6658502c98c386c0207
=======
p.Ks = 500;
>>>>>>> adding_folders

%Link masses
valM.mW = .42972;
valM.cm1 = .66162;
valM.cm2 = .9061;
valM.mB = 5.63;
p.valM = valM;
p.M = [valM.mW, valM.cm1, valM.cm2, valM.mB];

%CAD values
p.mK = .04; %currently not used
p.mH = .4;  %currently not used
p.R = 0.06; %radius of wheel

%Link lengths
<<<<<<< HEAD
<<<<<<< HEAD
valL.L1 = .15;
valL.L2 = .15;
=======
valL.L1 = .160;
valL.L2 = .200;
>>>>>>> b6f1ecc8a3fb2eda4e78b6658502c98c386c0207
=======
valL.L1 = .15;
valL.L2 = .15;
>>>>>>> adding_folders
valL.L3 = .4;
p.valL = valL;
p.L = [valL.L1, valL.L2, valL.L3,p.R];

%Linearization angle intialization
p.use_joints = false;
p.theta1_num = 0;
p.theta2_num = 0;
<<<<<<< HEAD
<<<<<<< HEAD
p.thetaHip_num = 0;
=======
p.theta3_num = 0;
>>>>>>> b6f1ecc8a3fb2eda4e78b6658502c98c386c0207
=======
p.thetaHip_num = 0;
>>>>>>> adding_folders

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