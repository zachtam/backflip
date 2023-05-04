import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

def unitvec(th):
    return np.array([[np.cos(th)],
                     [np.sin(th)]])

class Robot:
    # q = [back_hip, back_knee, front_hip, front_knee]
    # pose = [x, y, theta]
    thigh_length = 0.21
    shin_length = 0.19
    body_length = 0.38
    com2back = 0.19 # dist from back hip to com along back
    com2front = body_length - com2back

    def __init__(self, q=np.zeros(7)):
        self.q_ = np.array(q).reshape((7,))

    @property
    def q(self):
        return self.q_

    @property
    def joint_angles(self):
        return self.q_[3:]
    
    @property
    def pose(self):
        return self.q_[:3]
    
    @property
    def pos(self):
        return self.q_[:2]
    
    @property
    def theta(self):
        return self.q_[2]
    
    def joint_positions(self, plotorder=False):
        back_hip = self.pos - self.com2back*unitvec(self.q_[2])
        back_knee = back_hip + self.thigh_length*unitvec(self.q_[3])
        back_foot = back_knee + self.shin_length*unitvec(self.q_[3] + self.q_[4])
        front_hip = self.pos + self.com2front*unitvec(self.q_[2])
        front_knee = front_hip + self.thigh_length*unitvec(self.q_[5])
        front_foot = front_knee + self.shin_length*unitvec(self.q_[5] + self.q_[6])
        if plotorder:
            return np.hstack((back_foot, back_knee, back_hip, front_hip, front_knee, front_foot))
        return np.hstack((back_hip, back_knee, back_foot, front_hip, front_knee, front_foot))

    def goto(self, q):
        self.q_ = q
    
    def draw_frame(self, jp, ax):
        ax.artists = []
        ax.lines = [ax.lines[0]]
        return [ax.scatter(jp[0], jp[1], c='y')] + ax.plot(jp[0], jp[1], c='m')

    def animate(self, qs):
        # qs should be nx4
        jps = []
        for q in qs:
            self.goto(q)
            jps.append(self.joint_positions(plotorder=True))
        fig, ax = plt.subplots(1,1)
        f = lambda jp: self.draw_frame(jp, ax)
        ax.set_xlim(-1, 1)
        ax.set_ylim(-0.5, 1.5)
        ax.plot([-1, 1], [0, 0], c='k')
        anim = FuncAnimation(fig, f, frames=jps, interval=1, blit=True)
        
        plt.show()