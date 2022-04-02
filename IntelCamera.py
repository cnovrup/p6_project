# -*- coding: utf-8 -*-
"""
Created on Mon Mar 28 11:08:50 2022

@author: ctn
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import math

class Camera:
    def __init__(self, width, height):
        #self.intrinsics = (960.5, 960.7) #()
        self.fx = 960.5
        self.fy = 960.7
        
        self.width = width
        self.height = height
        
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.rotmat = rs.extrinsics().rotation
        
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)
        
        self.profile = self.pipeline.start(self.config)
        self.rgb_profile = self.profile.get_stream(rs.stream.color)
        self.intr = self.rgb_profile.as_video_stream_profile().get_intrinsics()
        self.fx = self.intr.fx
        self.fy = self.intr.fy
        self.u0 = self.intr.ppx
        self.v0 = self.intr.ppy
        
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_sensor.set_option( rs.option.min_distance, 0)
        self.depth_scale = self.depth_sensor.get_depth_scale()
        
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        
    def get_frames(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()
        
        if not aligned_depth_frame or not color_frame:
            return 0
        
        #depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        return aligned_depth_frame, color_image
    
    def get_dist(self, depth_frame, x, y):
        #return depth_image[int(y)][int(x)]*self.depth_scale
        return depth_frame.get_distance(x,y)
    
    def get_angles(self, x_px:int, y_px:int, w_px:int, h_px:int) -> float:
        xc = w_px/2
        yc = h_px/2
        
        th_x = math.atan((x_px - xc)/self.fx)
        th_y = math.atan((yc - y_px)/self.fy)

        return th_x, th_y
    
    def get_cart(self, theta, phi, rho):
        phi = (1/2*math.pi) - phi
        x = rho*math.sin(phi)*math.cos(theta)
        y = rho*math.sin(phi)*math.sin(theta)
        z = rho*math.cos(phi)
        return x,y,z
        
    def get_depth_color(self, depth_frame):
        depth_image = np.asanyarray(depth_frame.get_data())
        return cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    
    def get_intr(self):
        return self.intr
        
    def stop_stream(self):
        self.pipeline.stop()
