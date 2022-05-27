# -*- coding: utf-8 -*-
"""
Created on Fri May 13 13:20:31 2022

@author: ctn
"""
from custom_utils import saturation
import math
import numpy as np

class Controller:
    def __init__(self, Kp_lin, Kp_ang, max_ang_vel, max_lin_vel):
        self.Kp_lin = Kp_lin
        self.Kp_ang = Kp_ang
        self.max_ang_vel = max_ang_vel
        self.max_lin_vel = max_lin_vel
        
    def get_control(self, X, e_theta_x):
        lin_speed_x = saturation(self.Kp_lin*float(X[0]), self.max_lin_vel, -self.max_lin_vel)
        lin_speed_y = saturation(self.Kp_lin*float(X[1]), self.max_lin_vel, -self.max_lin_vel)
        lin_speed_z = saturation(self.Kp_lin*float(X[2]), self.max_lin_vel, -self.max_lin_vel)
        ang_speed_x = saturation(e_theta_x, self.max_ang_vel, -self.max_ang_vel)
        y_angle = np.arccos(float(X[2])/(np.linalg.norm(np.matrix([[float(X[0])], [0], [float(X[2])]]))*np.linalg.norm(np.matrix([[0],[0],[1]]))))
        y_angle = math.degrees(math.copysign(y_angle, float(X[0])))
        #print(y_angle, float(X[0]), sep='\t')
        ang_speed_y = saturation(self.Kp_ang*y_angle, self.max_ang_vel, -self.max_ang_vel)
        
        return lin_speed_x, lin_speed_y, lin_speed_z, ang_speed_x, ang_speed_y
    
