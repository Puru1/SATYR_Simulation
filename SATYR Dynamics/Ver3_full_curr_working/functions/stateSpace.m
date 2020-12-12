function [A,B] = stateSpace(angle_of_linearization)

    if strcmp('0',angle_of_linearization)
        A = [[ 0, -278.45297414971574948715438817291,  68.796787214032890539339645999598, -0.22538714919273273555844059576296, 0, 0, 0, 0]
              [ 0,  719.95075276170892955037545947012, -4601.9248507507302201736592119249,   15.076499425518700576150146839469, 0, 0, 0, 0]
              [ 0, -864.00484615829777455461857303789,  9105.5498573654484282830115658177,  -88.106777874296597018272684626165, 0, 0, 0, 0]
              [ 0,  146.41392690025664910990472773082,  -4577.401495209919841913248981269,   365.62235340494537645620730830702, 0, 0, 0, 0]];
          
        B = [[  8.4149635231299046587599522983132, -5.6079444067747026849065760194133, 0.95032008915813523387258821598101]
          [ -80.962379282135309222111507954637,  134.16032947485722056146156698534, -63.568399216939359258561563555742]
          [  145.37621828840662593127471902416,  -258.7014497669145493472796081115,  130.52866890736800476395413929988]
          [ -65.469039395255629726306739987704,  130.52866890736800476395413929988,  -91.80718463469948318505328307047]]; 
    
    elseif strcmp('10',angle_of_linearization)
        A = [[ 0, -119.78694323092521466849289804376, -45.281294368523600983471545611873, -0.73529500707573484460423215558006, 0, 0, 0, 0]
             [ 0,  446.91754871402666659782818622597, -121.95274308761528566397144305576,  -1.9803153673505974646715886197314, 0, 0, 0, 0]
             [ 0,  55.903588602784461050025195660459,  850.53443326354896919911881233599,   -84.57467533828989390947395143985, 0, 0, 0, 0]
             [ 0, -495.58201155163874564417516516406,  -718.0922853081699739548650918045,   172.53828475305294647372308328117, 0, 0, 0, 0]];

        B = [[  17.392680694942143623206277227478,  0.3199602528121242879073380258508, -2.8364287457801550611074376570601]
             [ -70.688272492183605994582915080268,  27.280980143175225526536606944238, -7.6391426292996286630497702801062]
             [  20.881775086932739768389846427222, -87.693013123582864575588581349425,  65.177672493847730493201443335987]
             [  49.089432286303472559098982861096,  65.177672493847730493201443335987, -67.260769578876199815806116750557]];

    elseif strcmp('45',angle_of_linearization)   
        A = [[ 0, -250.52339790220460986665344565639, -143.70888538536210120426682149758,  0.47080884591170455017734653576195, 0, 0, 0, 0]
                   [ 0,  313.18870770208615629173898095725,  170.48326199576656086811870866188, -0.55852515738505094432547563240001, 0, 0, 0, 0]
                   [ 0,  60.693327619618041196833121012915,  52.127350526312583038932924390923,  -47.073155503042170273856188699203, 0, 0, 0, 0]
                   [ 0, -380.00681340032948335532568443007, -226.25732584563162461973951835742,   282.93723198335130028337179509053, 0, 0, 0, 0]];
               
         B = [[   5.4122558776606761473482709619742,  0.39393853942645739007020922547015,  -2.466487419856689507690013059702]
           [  -6.7814285451506462215155640283271, -0.16301976470873658530521714386272,  2.9260182478008502886307352607103]
           [ -0.95089684356165136544563559480303,   -1.506655090423858370388777465817,  5.6443951866410170704287186003251]
           [   7.8589930875142293040107613801143,   5.6443951866410170704287186003251, -32.460810338729763738822993598627]];         
    end
    
A = [[zeros(4,4) eye(4)]; A]; 
B = [zeros(4,3); B];                      
end