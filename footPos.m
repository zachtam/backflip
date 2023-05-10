function [rFoot, fFoot] = footPos(q, thigh_length, shin_length, body_length)
    rFoot = [q(1) - 0.5*body_length*cos(q(3)) + thigh_length*cos(q(3)+q(4)) + shin_length*cos(q(3)+q(4)+q(5));
             q(2) - 0.5*body_length*sin(q(3)) + thigh_length*sin(q(3)+q(4)) + shin_length*sin(q(3)+q(4)+q(5))];
    fFoot = [q(1) - 0.5*body_length*cos(q(3)) + thigh_length*cos(q(3)+q(6)) + shin_length*cos(q(3)+q(6)+q(7));
             q(2) - 0.5*body_length*sin(q(3)) + thigh_length*sin(q(3)+q(6)) + shin_length*sin(q(3)+q(6)+q(7))];
end