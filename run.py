from planarquadsim import Robot
import numpy as np

r = Robot()
r.animate(np.linspace((0, 0, 0, -0.5, 0.3, -0.5, 0.3), (0, 0, 0, -2.0, 0.3, -2.0, 0.3)))