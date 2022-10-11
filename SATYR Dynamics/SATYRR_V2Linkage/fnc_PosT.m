function [PosT] = fnc_PosT(q,L)

PosT = zeros(3,1);

<<<<<<< HEAD
<<<<<<< HEAD
  PosT(1,1)=q(1) + L(1)*sin(q(2) + q(3)) + L(3)*sin(q(3)) - L(2)*sin(q(2) - q(3));
  PosT(2,1)=0;
  PosT(3,1)=L(1)*cos(q(2) + q(3)) + L(3)*cos(q(3)) + L(2)*cos(q(2) - q(3));
=======
  PosT(1,1)=q(1) - L(1)*sin(q(2)/2 - q(3)) + L(3)*sin(q(3)) + L(2)*sin(q(2)/2 + q(3));
  PosT(2,1)=0;
  PosT(3,1)=L(1)*cos(q(2)/2 - q(3)) + L(3)*cos(q(3)) + L(2)*cos(q(2)/2 + q(3));
>>>>>>> b6f1ecc8a3fb2eda4e78b6658502c98c386c0207
=======
  PosT(1,1)=q(1) + L(1)*sin(q(2) + q(3)) + L(3)*sin(q(3)) - L(2)*sin(q(2) - q(3));
  PosT(2,1)=0;
  PosT(3,1)=L(1)*cos(q(2) + q(3)) + L(3)*cos(q(3)) + L(2)*cos(q(2) - q(3));
>>>>>>> adding_folders

 