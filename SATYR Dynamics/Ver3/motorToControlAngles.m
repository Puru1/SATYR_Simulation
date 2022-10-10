function [theta1_c,theta2_c,theta3_c] = motorToControlAngles(theta,pitch)
    theta3_c = - theta(3);
    theta2_c = - theta(2);
    theta1_c = theta(2) + theta(3) - pitch;
end