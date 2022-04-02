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

angles = [101.5*math.pi/180, -4.9*math.pi/180, 102.1*math.pi/180]
#angles = [0, 0, 0]
distances = [0.4, 0.08, -0.02]
p0 = np.array([0.397, 0.04, 0.126])
point_base_hand(angles, distances, p0)
#print(point_base_hand(angles, distances, p0))