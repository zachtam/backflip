import matplotlib.pyplot as plt
import numpy as np

class Robot:
    # q = [front_hip, front_knee, back_hip, back_knee]
    # pose = [x, y, theta]
    thigh_length = 0.21
    shin_length = 0.19
    body_length = 0.38
    com_position = 0.0

    def __init__(q, pose):
        self.q_ = q
        self.pose_ = pose

    def joint_angles():
        return self.q_