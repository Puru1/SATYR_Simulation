function CoM = comCalc(q,L)

%     PosCm1 = fnc_PosCM1(q,L) -  fnc_PosK(q,L);
%     PosCm2 = fnc_PosCM2(q,L) -  fnc_PosH(q,L);
%     PosCmR = fnc_PosCoM_R(q,L) -  fnc_PosT(q,L);
%     PosCm1= [fnc_PosCM1(q,L);1] - fnc_HTMBK(q,L)*[fnc_PosCM1(q,L);1];
%     PosCm2= [fnc_PosCM2(q,L);1] - fnc_HTMBH(q,L)*[fnc_PosCM2(q,L);1];
%     PosCmR= [fnc_PosCoM_R(q,L);1] - fnc_HTMBT(q,L)*[fnc_PosCoM_R(q,L);1];
%     PosCm1= inv(fnc_HTMBK(q,L))*[fnc_PosCM1(q,L);1];
    PosCm1= inv(fnc_HTMBK(q,L))*(fnc_HTMBcm1(q,L))
    PosCm2= inv(fnc_HTMBH(q,L))*[fnc_PosCM2(q,L);1];
    PosCmR= inv(fnc_HTMBT(q,L))*[fnc_PosCoM_R(q,L);1];

    CoM = [PosCm1(1:3,4), PosCm2(1:3), PosCmR(1:3)]
end 
