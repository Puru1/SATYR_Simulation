function [PosCM2] = fnc_PosCM2(q,L)

PosCM2 = zeros(3,1);

<<<<<<< HEAD
<<<<<<< HEAD
  PosCM2(1,1)=q(1) + L(1)*sin(q(2) + q(3)) - (9*L(2)*sin(q(2) - q(3)))/10;
  PosCM2(2,1)=0;
  PosCM2(3,1)=L(1)*cos(q(2) + q(3)) + (9*L(2)*cos(q(2) - q(3)))/10;
=======
  PosCM2(1,1)=q(1) - L(1)*sin(q(2)/2 - q(3)) + (9*L(2)*sin(q(2)/2 + q(3)))/10;
  PosCM2(2,1)=0;
  PosCM2(3,1)=L(1)*cos(q(2)/2 - q(3)) + (9*L(2)*cos(q(2)/2 + q(3)))/10;
>>>>>>> b6f1ecc8a3fb2eda4e78b6658502c98c386c0207
=======
  PosCM2(1,1)=q(1) + L(1)*sin(q(2) + q(3)) - (9*L(2)*sin(q(2) - q(3)))/10;
  PosCM2(2,1)=0;
  PosCM2(3,1)=L(1)*cos(q(2) + q(3)) + (9*L(2)*cos(q(2) - q(3)))/10;
>>>>>>> adding_folders

 