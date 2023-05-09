from planarquadsim import Robot
import numpy as np

r = Robot()
# r.animate(np.linspace((0, 0, 0, -0.5, 0.3, -0.5, 0.3), (0, 0, 0, -2.0, 0.3, -2.0, 0.3)))
q0 = [0, 0, 0, -(7/8)*np.pi, (3/4)*np.pi, -(7/8)*np.pi, (3/4)*np.pi]
r.goto(q0)
foot_y = r.joint_positions()[2, 1]
q0[1] = -foot_y
# r.animate([q0])

qT = [0, 0, -1.9*np.pi, -(9/16)*np.pi, (1/8)*np.pi, -(9/16)*np.pi, (1/8)*np.pi]
r.goto(qT)
foot_y = r.joint_positions()[2, 1]
qT[1] = -foot_y
# r.animate([qT])

# Testing code

# Paper uses more complicated dynamics that include actuator info
# We can just use the matrices that are obtained here


# Import libraries
import casadi as cd
import numpy as np
import matplotlib.pyplot as plt


N = 80
opti = cd.Opti()

# X = opti.variable(7+7+7+(2*2), N+1)
q = opti.variable(7 * (N+1))
q_dot = opti.variable(7 * (N+1))
q_ddot = opti.variable(7 * (N+1)) # unused?
fk = opti.variable((2*2), (N+1)) # xy for each of 2 feet
u = opti.variable((2*2), (N+1))

# T = opti.variable()
opti.minimize(0)

fmin = 1 # TODO: I made this up
mu = 0.75 # TODO: I made this up

f = None # specify system dynamics

### Constraints

# TODO: Euler integration of manipulator equation constraint

robot = Robot(casadi=True)

opti.subject_to(q[:7]==q0) # Initial position of robot
opti.subject_to(q_dot[:7]==0) # Initial velocity of the robot
opti.subject_to(q[-7:]==qT) # Landing position as described in paper

# unilateral constraint
opti.subject_to(fk[1] > 0)
opti.subject_to(fk[3] > 0)

# ground reaction forces must be in friction cone
opti.subject_to(fk[0] <= mu*fk[1])
opti.subject_to(-fk[0] <= mu*fk[1])
opti.subject_to(fk[2] <= mu*fk[3])
opti.subject_to(-fk[2] <= mu*fk[3])

for t in range(N+1):
    robot.goto(q[7*t:7*(t+1)])
    jps = robot.joint_positions()
    # knees above ground
    opti.subject_to(jps[1, 1] > 0)
    opti.subject_to(jps[4, 1] > 0)
    # front foot should not get too close to the rear foot/hip
    opti.subject_to(cd.norm_2(jps[5] - jps[2]) > 0.03) # SOC constraint
    opti.subject_to(cd.norm_2(jps[5] - jps[0]) > 0.03) # SOC constraint
    # zero contact forces when foot not on the ground
    opti.subject_to(cd.fabs(jps[2, 1] * fk[0, t]) < 1e-6) # ground position 0 or both of these contact forces 0
    opti.subject_to(cd.fabs(jps[2, 1] * fk[1, t]) < 1e-6)
    opti.subject_to(cd.fabs(jps[5, 1] * fk[2, t]) < 1e-6)
    opti.subject_to(cd.fabs(jps[5, 1] * fk[3, t]) < 1e-6)
    # normal force >= fmin to prevent slipping
    opti.subject_to(1000*cd.fabs(jps[2, 1]) + fk[1, t] >= fmin)
    opti.subject_to(1000*cd.fabs(jps[5, 1]) + fk[3, t] >= fmin)
    # torque limits (see thesis page 29)
    opti.subject_to(u[:, t] < 18)
    # velocity dependent torque limits
    # line through (15, 18) and (40, 0) of (q_dot, u)
    # q_dot = (-25/18)u + 40
    # u = (-18/25)*(q_dot - 40)
    opti.subject_to(u[:, t] <= (-18/25)*(q_dot[7*t+3:7*(t+1)] - 40))
    # feet on the ground has zero acceleration
    # dpos/dt = dpos/dq * qdot
    # d(dpos/dt)/dt = d/dt (dpos/dq * qdot) = dpos/dq * qddot + d/dt(dpos/dq) * qdot
    # = dpos/dq * qddot + d(dpos/dq)/dq * qdot^2
    # also equals d(dpos/dq * qdot)/dq * qdot
    back_foot_acceleration = cd.mtimes(cd.jacobian(cd.mtimes(cd.jacobian(jps[2], q), q_dot), q), q_dot)
    front_foot_acceleration = cd.mtimes(cd.jacobian(cd.mtimes(cd.jacobian(jps[5], q), q_dot), q), q_dot)
    opti.subject_to(-1000*cd.fabs(jps[2, 1]) + cd.fabs(back_foot_acceleration) < 1e-6)
    opti.subject_to(-1000*cd.fabs(jps[5, 1]) + cd.fabs(front_foot_acceleration) < 1e-6)

# opti.subject_to(T>=0)

# opti.set_initial(T, 1)
opti.solver('ipopt')
try:
    sol = opti.solve()
except Exception as e:
    print(e)

r = Robot()
qstar = opti.debug.value(q)
for t in [0]: # range(N+1):
    r.goto(qstar[7*t:7*(t+1)])
    jps = r.joint_positions()
    # print(jps)
r.animate(qstar.reshape((-1, 7)))