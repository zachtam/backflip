syms x y x_dot y_dot qb qb_dot q1 q2 q3 q4 q1_dot q2_dot q3_dot q4_dot;



q_act = [q1;q2;q3;q4];

m =9;



half_body_length = .38;

vbody = [x_dot; y_dot];

T = 0.5*m*(norm(vbody)^2) + (1/24)*m*(half_body_length*2)^2 * (qb_dot)^2;
U = m*g*y;
q = [x; y; qb;q1;q2;q3;q4;];
dq =[x;y;qb_dot;q1_dot;q2_dot; q3_dot; q4_dot];

[D, C, G, B] = LagrangianDynamics(T, U, q, dq, q_act)





