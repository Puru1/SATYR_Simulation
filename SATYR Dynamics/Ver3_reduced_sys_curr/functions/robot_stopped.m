function [zeroCrossing,isterminal,direction] = robot_stopped(t,X,p)


%Check to see if wheel angular velocity is nearly stopped. If so change
%flag from positive to negative indicating completion.
if X(5) < .01
    done = -.01;
else
    done = 1;
end

zeroCrossing =  done;
isterminal   =  1;
direction    =  -1;
