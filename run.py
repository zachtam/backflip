from planarquadsim import Robot
import numpy as np

r = Robot()
# r.animate(np.linspace((0, 0, 0, -0.5, 0.3, -0.5, 0.3), (0, 0, 0, -2.0, 0.3, -2.0, 0.3)))
q0 = [0, 0, 0, -(7/8)*np.pi, (3/4)*np.pi, -(7/8)*np.pi, (3/4)*np.pi]
r.goto(q0)
foot_y = r.joint_positions()[1, 2]
q0[1] = -foot_y
r.animate([q0])

qT = [0, 0, 0, -(9/16)*np.pi, (1/8)*np.pi, -(9/16)*np.pi, (1/8)*np.pi]
r.goto(qT)
foot_y = r.joint_positions()[1, 2]
qT[1] = -foot_y
r.animate([qT])

# # Testing code

# # Paper uses more complicated dynamics that include actuator info
# # We can just use the matrices that are obtained here


# # Import libraries
# import casadi as cd
# import numpy as np
# import matplotlib.pyplot as plt


# N = 80
# opti = cd.Opti()

# X = cd.variable(25, N+1)
# q = X[:7, :]
# q_dot = X[7:14, :]
# q_ddot = X[14:21, :]
# f = X[21:25, :]

# T = opti.variable()
# opti.minimize(0)

# f = None # specify system dynamics

# ### Constraints

# # TODO: Euler integration of manipulator equation constraint

# opti.subject_to(q[0]==?) # Initial position of robot
# opti.subject_to(q_dot[0]==0) # Initial velocity of the robot
# opti.subject_to(q[N+1]==) # Landing position as described in paper
# opti.subject_to() # Knees above ground

# opti.subject_to(T>=0)

# opti.set_initial(T, 1)
# opti.solver('ipopt')
# sol = opti.solve()