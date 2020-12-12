function p = getParams()
    p.g = 9.81;
    p.mW = .42972;
    p.mCM = 15.513;
    p.L1 = .1414;
    p.R = .05;
    p.theta1_num = 0;%angle of linearization
    
    IW = [[.000423   0     0    ];
      [   0   .00075   0    ];
      [   0      0  .000423 ]];

    ICM = [[.23368    0     0    ];
      [   0   .19732   0    ];
      [   0      0   .06132 ]];
  
    p.IW = IW;
    p.ICM = ICM;

end