syms q1 q2 q3 q4 q1_dot q2_dot q3_dot q4_dot;



q_act = [q1;q2;q3;q4];

q1vec = [0;0;q1];
q2vec = [0;0;q2];
q3vec = [0;0;q3];
q4vec = [0;0;q4];

q1_dot_vec = [0;0;q1_dot];
q2_dot_vec = [0;0;q2_dot];
q3_dot_vec = [0;0;q3_dot];
q4_dot_vec = [0;0;q4_dot];


m = 3;
g = 9.81;

half_body_length = 5;
l1_length = 3;
l2_length = 3;
l3_length = 3;
l4_length = 3;

qb = asin((l3_length*sin(q3) + l4_length*sin(q4) - l1_length*sin(q1) - l2_length*sin(q2))/(half_body_length *2));

lL = [-half_body_length*cos(qb); -half_body_length*sin(qb);0];
lR = [+half_body_length*cos(qb); +half_body_length*sin(qb);0];

l1 = [-l1_length*cos(q1+qb); -l1_length*sin(q1+qb);0];
l2 = [-l2_length*cos(q2+q1+qb); -l2_length*sin(q2+q1+qb);0];
l3 = [-l3_length*cos(q3+qb); -l3_length*sin(q3+qb);0];
l4 = [-l4_length*cos(q4+q3+qb); -l4_length*sin(q4+q3+qb);0];

qb_vec = [0;0;qb];
x = half_body_length*cos(qb) + l1_length*cos(q1) +l2_length*cos(q2);
y = half_body_length*sin(qb) + l1_length*sin(q1) + l2_length*sin(q2);
%pinv(skew(transpose(lL - lR)))
qb_dot_vec = (pinv(skewf(transpose(lL - lR))))*((skewf(q3vec) * l3) + (skewf(q4vec) * l4) - (skewf(q1vec) * l1) - (skewf(q2vec) * l2));
vbod = 0 - (skewf(qb_dot_vec)*lR) - (skewf(q3_dot_vec) * l3) - (skewf(q4_dot_vec)*l4);
x_dot = vbod(1);
y_dot = vbod(2);






vbody = [x_dot; y_dot]

qb_dot = qb_dot_vec(3)

T = 0.5*m*(norm(vbody)^2) %+ (1/24)*m*(half_body_length*2)*(qb_dot^2)
U = m*g*y
q = [q1;q2;q3;q4];
dq =[q1_dot;q2_dot; q3_dot; q4_dot];

%[D, C, G, B] = LagrangianDynamics(T, U, q, dq, q_act)





