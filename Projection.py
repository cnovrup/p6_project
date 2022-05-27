# -*- coding: utf-8 -*-
"""
Created on Fri May 13 13:10:40 2022

@author: ctn
"""

import math
import numpy as np
from custom_utils import saturation

class Projection:
    def __init__(self, cam_obj, cam_y, cam_z, tilt, width, height):
        self.Rs = np.matrix([[-1,0,0],[0,-1,0],[0,0,1]])
        self.Rx = np.matrix([[1, 0, 0], [0, math.cos(math.radians(-tilt)), -math.sin(math.radians(-tilt))],[0, math.sin(math.radians(-tilt)), math.cos(math.radians(-tilt))]])
        self.R = self.Rs * self.Rx
        self.K = np.matrix([[cam_obj.fx,0,cam_obj.u0],[0, cam_obj.fy, cam_obj.v0],[0,0,1]])
        self.t = np.matrix([[0],[cam_y],[cam_z]])
        self.M = self.K * np.c_[np.transpose(self.R), -np.transpose(self.R)*self.t]
        self.Minv = np.linalg.pinv(self.M)
        self.width = width
        self.height = height
        
    def get_gripper_proj(self):
        gripper = self.M*np.matrix([[0],[0],[0],[1]])
        gripper = gripper/float(gripper[2])
        return gripper
    
    def get3d_point(self, center, d):
        X = self.Minv * np.matrix([[center[0]], [center[1]], [1]])
        X = X*(1/float(X[3]))
        X = X[0:3]
        l = d/np.linalg.norm(X+self.t)
        X = self.t+(X*l)
        return X
    
    def get_proj(self, X):
        est_dot = self.M*np.matrix([[float(X[0])],[float(X[1])],[float(X[2])],[1]])
        est_dot = est_dot/float(est_dot[2])
        est_dot_x = saturation(int(est_dot[0]), self.width-1, 0)
        est_dot_y = saturation(int(est_dot[1]), self.height-1, 0)
        return est_dot_x, est_dot_y
