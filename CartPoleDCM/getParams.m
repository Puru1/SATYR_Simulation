function p = getParams()
    p.g = 9.81;
    %WIP params where mW/base << mCM
    p.mW = .42972;
    p.mCM = 5.663 + 1.5;
    
    %Cart-pole params where mW/base >> mCM
    %p.mCM = .42972;
    %p.mW = 5.663 + 1.5;
    
    p.L1 = .35329;
    %p.L1 = .141;
    p.R = .06;
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