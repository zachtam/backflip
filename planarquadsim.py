import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib
import casadi as cd

def unitvec(th, casadi=False):
    if casadi:
        v = cd.MX(2, 1)
        v[0, 0] = np.cos(th)
        v[1, 0] = np.sin(th)
        return v
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

    def __init__(self, q=np.zeros(7), casadi=False):
        if casadi:
            self.q_ = q
        else:
            self.q_ = np.array(q).reshape((7,))
        self.casadi = casadi

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
        back_hip = self.pos.reshape((2, 1)) - self.com2back*unitvec(self.q_[2], self.casadi)
        back_knee = back_hip + self.thigh_length*unitvec(self.q_[2] + self.q_[3], self.casadi)
        back_foot = back_knee + self.shin_length*unitvec(self.q_[2] + self.q_[3] + self.q_[4], self.casadi)
        front_hip = self.pos.reshape((2, 1)) + self.com2front*unitvec(self.q_[2], self.casadi)
        front_knee = front_hip + self.thigh_length*unitvec(self.q_[2] + self.q_[5], self.casadi)
        front_foot = front_knee + self.shin_length*unitvec(self.q_[2] + self.q_[5] + self.q_[6], self.casadi)
        if plotorder:
            return np.hstack((back_foot, back_knee, back_hip, front_hip, front_knee, front_foot))
        if self.casadi:
            return cd.horzcat(back_hip, back_knee, back_foot, front_hip, front_knee, front_foot).T
        return np.hstack((back_hip, back_knee, back_foot, front_hip, front_knee, front_foot)).T

    def goto(self, q):
        if self.casadi:
            self.q_ = q
        else:
            self.q_ = np.array(q).reshape((7,))

    def animate(self, qs):
        # qs should be nx4
        jps = []
        for q in qs:
            self.goto(q)
            jps.append(self.joint_positions(plotorder=True))
        fig, ax = plt.subplots(1,1)
        ax.set_xlim(-1, 1)
        ax.set_ylim(-0.5, 1.5)
        ax.plot([-1, 1], [0, 0], c='k')

        lines = matplotlib.lines.Line2D([0], [0],
            color='m', marker='o',
            markerfacecolor='y', markeredgecolor='y')
        ax.add_artist(lines)
        
        def draw_frame(jp):
            lines.set_data(jp[0], jp[1])
            return lines,
        
        anim = FuncAnimation(fig, draw_frame, frames=jps, interval=20, blit=True, repeat=False)
        
        plt.show()
    