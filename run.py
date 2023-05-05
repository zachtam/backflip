from planarquadsim import Robot
import numpy as np

# r = Robot()
# r.animate(np.linspace((0, 0, 0, -0.5, 0.3, -0.5, 0.3), (0, 0, 0, -2.0, 0.3, -2.0, 0.3)))

# Testing code

# Paper uses more complicated dynamics that include actuator info
# We can just use the matrices that are obtained here


# Import libraries
import casadi as cd
import numpy as np
import matplotlib.pyplot as plt


N = 80
opti = cd.Opti()

X = cd.variable(25, N+1)
q = X[:7, :]
q_dot = X[7:14, :]
q_ddot = X[14:21, :]
f = X[21:25, :]

T = opti.variable()
opti.minimize(0)

f = None # specify system dynamics

### Constraints

# TODO: Euler integration of manipulator equation constraint


opti.subject_to(q[0]==?) # Initial position of robot
opti.subject_to(q_dot[0]==0) # Initial velocity of the robot
opti.subject_to(q[N+1]==) # Landing position as described in paper
opti.subject_to() # Knees above ground

opti.subject_to(T>=0)

opti.set_initial(T, 1)
opti.solver('ipopt')
sol = opti.solve()