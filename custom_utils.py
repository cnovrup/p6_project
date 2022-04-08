# -*- coding: utf-8 -*-
"""
Created on Wed Mar 30 09:34:44 2022

@author: ctn
"""

import numpy as np
import math

base_vectors = np.array([[0,-1,0], [1,0,0], [0,0,1]])

def rotate_frame(alpha, phi, theta):
    Rxa = np.array([[1, 0, 0],[0, math.cos(alpha), -math.sin(alpha)], [0, math.sin(alpha), math.cos(alpha)]])
    Ryp = np.array([[math.cos(phi), 0, math.sin(phi)], [0, 1, 0], [-math.sin(phi), 0, math.cos(phi)]]);
    Rzt = np.array([[math.cos(theta), -math.sin(theta), 0], [math.sin(theta), math.cos(theta), 0], [0, 0, 1]])

    return Rxa @ Ryp @ Rzt

def point_base_hand(angles, distances, p0):
    # angles = [alpha_x, phi_y, theta_z] - radians
    # distances = [dx, dy, dz]
    # p0 = numpy vector
    point_from_camera = np.array([distances[0], distances[1], distances[2]])
    gripper_frame = base_vectors @ rotate_frame(angles[0], angles[1], angles[2])
    print(gripper_frame)
    P = gripper_frame @ point_from_camera + p0
    return P

def saturation(val, max_val, min_val):
    if(val > max_val):
        return max_val
    elif(val < min_val):
        return min_val
    else:
        return val