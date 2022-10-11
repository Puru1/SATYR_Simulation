function [PosCM1] = fnc_PosCM1(q,L)

PosCM1 = zeros(3,1);

<<<<<<< HEAD
<<<<<<< HEAD
  PosCM1(1,1)=q(1) + (2169*L(1)*sin(q(2) + q(3)))/10000;
  PosCM1(2,1)=0;
  PosCM1(3,1)=(2169*L(1)*cos(q(2) + q(3)))/10000;
=======
  PosCM1(1,1)=q(1) - (2169*L(1)*sin(q(2)/2 - q(3)))/10000;
  PosCM1(2,1)=0;
  PosCM1(3,1)=(2169*L(1)*cos(q(2)/2 - q(3)))/10000;
>>>>>>> b6f1ecc8a3fb2eda4e78b6658502c98c386c0207
=======
  PosCM1(1,1)=q(1) + (2169*L(1)*sin(q(2) + q(3)))/10000;
  PosCM1(2,1)=0;
  PosCM1(3,1)=(2169*L(1)*cos(q(2) + q(3)))/10000;
>>>>>>> adding_folders

 