% clear
%% Define symbolic variables for cofiguration variables and mechanical parameters

syms q1 q2 q3 x y real
syms dq1 dq2 dq3 dx dy real
syms u1 u2
syms lL lT mL mT JL JT g real

% Position Variable vector
q = [x;y;q1;q2;q3];

% Velocity variable vector
dq = [dx;dy;dq1;dq2;dq3];

% Inputs
tau = [u1;u2];

% parameters
params = [lL; lT; mL; mT; JL; JT; g];

% Parameter Values from Table 1
paramValues = [1;  % Leg Length
               0.5; % Torso Length
               5; % Leg mass
               10; % Torso mass
               0.5; % Leg Moment of inertia
               1; % Torso moment of inertia
               9.81]; % acc due to gravity

% # of Degrees of freedom
NDof = length(q);

% Given values for q and dq
qVal1 = [0.5;sqrt(3)/2;deg2rad(150);deg2rad(120);deg2rad(30)];
dqVal1 = [-0.8049;-0.4430;0.0938;0.9150;0.9298];

qVal2 = [0.3420;0.9397;deg2rad(170);deg2rad(20);deg2rad(30)];
dqVal2 = [-0.1225;-0.2369;0.5310;0.5904;0.6263];
%% Problem 1: Lagrangian Dynamics
%Find the CoM position of each link

% Torso
pComTorso = [x + lT/2*sin(q3);...
             y + lT/2*cos(q3)];

% Leg 1
pComLeg1 = [x - lL/2*cos(deg2rad(270) - (q1 + q3));...
            y - lL/2*sin(deg2rad(270) - (q1 + q3))];

% Leg 2
pComLeg2 = [x + lL/2*cos(q2 + q3 - deg2rad(90));...
            y - lL/2*sin(q2 + q3 - deg2rad(90))];

disp('CoM positions:')
P1 = double(subs([pComLeg1, pComLeg2, pComTorso], [params;q;dq], [paramValues;qVal1;dqVal1]))
P2 = double(subs([pComLeg1, pComLeg2, pComTorso], [params;q;dq], [paramValues;qVal2;dqVal2]))

% Find the CoM velocity of each link

% Torso
dpComTorso = simplify(jacobian(pComTorso, q)*dq);

% Leg 1
dpComLeg1 = simplify(jacobian(pComLeg1, q)*dq);

% Leg 2
dpComLeg2 = simplify(jacobian(pComLeg2, q)*dq);


disp('CoM velocitues:')
V1 = double(subs([dpComLeg1, dpComLeg2, dpComTorso], [params;q;dq], [paramValues;qVal1;dqVal1]))
V2 = double(subs([dpComLeg1, dpComLeg2, dpComTorso], [params;q;dq], [paramValues;qVal2;dqVal2]))

% Find the absolute angle associated with each link
% Torso
q3Absolute = q3;
% Leg 1
q1Absolute = q3 + q1;
% Leg 2
q2Absolute = q3 + q2;
%% Find absolute angular velocity associated with each link:
% Torso
dq3Absolute = dq3;
% Leg 1
dq1Absolute = dq3 + dq1;
% Leg 2
dq2Absolute = dq3 + dq2;

% Total Kinetic energy = Sum of kinetic energy of each link

% Torso
KETorso = 0.5*mT*dpComTorso(1)^2 + 0.5*mT*dpComTorso(2)^2 + 0.5*JT*dq3Absolute^2;

% Leg 1
KELeg1 = 0.5*mL*dpComLeg1(1)^2 + 0.5*mL*dpComLeg1(2)^2 + 0.5*JL*dq1Absolute^2;

% Leg 2
KELeg2 = 0.5*mL*dpComLeg2(1)^2 + 0.5*mL*dpComLeg2(2)^2 + 0.5*JL*dq2Absolute^2;

% Total KE
KE = simplify(KETorso + KELeg1 + KELeg2);

disp('Total kinetic energy:')
KEVal1 = double(subs(KE, [params;q;dq], [paramValues;qVal1;dqVal1]))
KEVal2 = double(subs(KE, [params;q;dq], [paramValues;qVal2;dqVal2]))

% Total potential energy = Sum of Potential energy of each link

% Torso
PETorso = mT*g*pComTorso(2);

%Leg 1
PELeg1 = mL*g*pComLeg1(2);

% Leg 2
PELeg2 = mL*g*pComLeg2(2);

% Total PE
PE = simplify(PETorso + PELeg1 + PELeg2);

disp('Total potential energy:')
PEVal1 = double(subs(PE, [params;q;dq], [paramValues;qVal1;dqVal1]))
PEVal2 = double(subs(PE, [params;q;dq], [paramValues;qVal2;dqVal2]))
% Lagrangian

L = KE - PE;

% Equations of Motion
EOM = jacobian(jacobian(L,dq), q)*dq - jacobian(L, q)' ;
EOM = simplify(EOM);

% Find the D, C, G, and B matrices

% Actuated variables
qActuated = [q1;q2];

% D, C, G, and B matrices
[D, C, G, B] = LagrangianDynamics(KE, PE, q, dq, qActuated);

% simplify and print out the results
disp('Symbolic Expressions for D, C, G, B matrices')
D = simplify(subs(D, params, paramValues))
C = simplify(subs(C, params, paramValues))
G = simplify(subs(G, params, paramValues))
B

disp('Numerical Values for D, C, G matrices for given configurations')
DVal1 = double(subs(D, [q;dq], [qVal1;dqVal1]))
CVal1 = double(subs(C, [q;dq], [qVal1;dqVal1]))
GVal1 = double(subs(G, [q;dq], [qVal1;dqVal1]))


DVal2 = double(subs(D, [q;dq], [qVal2;dqVal2]))
CVal2 = double(subs(C, [q;dq], [qVal2;dqVal2]))
GVal2 = double(subs(G, [q;dq], [qVal2;dqVal2]))

%% Problem 2: Change of Coordinates

% Transformation matrix:
T = [1 0 0 0 0;
     0 1 0 0 0;
     0 0 1 0 1;
     0 0 0 1 1;
     0 0 0 0 1];

d = [0;
     0;
     -pi;
     -pi;
     0];

% Position
qtildeVal1 = T*qVal1 + d
qtildeVal2 = T*qVal2 + d

% Velocity
dqtildeVal1 = T*dqVal1
dqtildeVal2 = T*dqVal2

% Dynamics
% For Problem 2(d), redo Problem 1 using absolute coordinates

DtildeVal1 = inv(T')*DVal1*inv(T)
CtildeVal1 = inv(T')*CVal1*inv(T)
GtildeVal1 = inv(T')*GVal1
BtildeVal1 = inv(T')*B


DtildeVal2 = inv(T')*DVal2*inv(T)
CtildeVal2 = inv(T')*CVal2*inv(T)
GtildeVal2 = inv(T')*GVal2
BtildeVal2 = inv(T')*B
