syms x y x_dot y_dot qb qb_dot q1 q2 q3 q4 q1_dot q2_dot q3_dot q4_dot real;

q = [x; y; qb;q1;q2;q3;q4;];
dq =[x_dot;y_dot;qb_dot;q1_dot;q2_dot; q3_dot; q4_dot];
q_act = [q1;q2;q3;q4];

m = 7;
mFoot = 1;
g = 9.81;

thigh_length = 0.21;
shin_length = 0.19;
body_length = 0.38;

vbody = [x_dot; y_dot];

[rFoot, fFoot] = footPos(q, thigh_length, shin_length, body_length);
rFootVel = simplify(jacobian(rFoot, q) * dq);
fFootVel = simplify(jacobian(fFoot, q) * dq);
footT = 0.5*mFoot*(rFootVel(1, 1)^2 + rFootVel(2, 1)^2 + fFootVel(1, 1)^2 + fFootVel(2, 1)^2);
T = 0.5*m*(x_dot^2 + y_dot^2) + (1/24)*m*(body_length)^2 * (qb_dot)^2 + footT;
U = m*g*y  + mFoot*g*rFoot(2, 1) + mFoot*g*fFoot(2, 1);

[D, C, G, B] = LagrangianDynamics(simplify(T), simplify(U), q, dq, q_act)





