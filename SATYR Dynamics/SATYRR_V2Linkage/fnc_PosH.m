function [PosH] = fnc_PosH(q,L)

PosH = zeros(3,1);

<<<<<<< HEAD
<<<<<<< HEAD
  PosH(1,1)=q(1) + L(1)*sin(q(2) + q(3)) - L(2)*sin(q(2) - q(3));
  PosH(2,1)=0;
  PosH(3,1)=L(1)*cos(q(2) + q(3)) + L(2)*cos(q(2) - q(3));
=======
  PosH(1,1)=q(1) - L(1)*sin(q(2)/2 - q(3)) + L(2)*sin(q(2)/2 + q(3));
  PosH(2,1)=0;
  PosH(3,1)=L(1)*cos(q(2)/2 - q(3)) + L(2)*cos(q(2)/2 + q(3));
>>>>>>> b6f1ecc8a3fb2eda4e78b6658502c98c386c0207
=======
  PosH(1,1)=q(1) + L(1)*sin(q(2) + q(3)) - L(2)*sin(q(2) - q(3));
  PosH(2,1)=0;
  PosH(3,1)=L(1)*cos(q(2) + q(3)) + L(2)*cos(q(2) - q(3));
>>>>>>> adding_folders

 