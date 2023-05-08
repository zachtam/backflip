syms x y q1 q2 q3 q4 qb x_dot y_dot q1_dot q2_dot q3_dot q4_dot qb_dot ;

q = [x;y;q1;q2;q3;q4;qb];
dq =[x_dot;y_dot;q1_dot;q2_dot;q3_dot;q4_dot; qb_dot];

q_act = [q1;q2;q3;q4];

m = 3;
g = 9.81;

half_body_length = 5;
l1_length = 3;
l2_length = 3;
l3_length = 3;
l4_length = 3;

lL = [x-half_body_length*cos(qb); y-half_body_length*sin(qb)];
lR = [x+half_body_length*cos(qb); y+half_body_length*sin(qb)];

l1 = lL + [-l1_length*cos(q1); -l1_length*sin(q1)];
l2 = l1 + [-l2_length*cos(q2); -l2_length*sin(q2)];
l3 = lR + [-l3_length*cos(q3); -l3_length*sin(q3)];
l4 = l3 + [-l4_length*cos(q4); -l4_length*sin(q4)];

vbody = [x_dot; y_dot];

T = 0.5*m*(norm(vbody)^2) + (1/24)*m*(half_body_length*2)*(qb_dot^2);
U = m*g*y;

[D, C, G, B] = LagrangianDynamics(T, U, q, dq, q_act)


