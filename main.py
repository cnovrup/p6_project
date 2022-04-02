# -*- coding: utf-8 -*-
"""
Created on Mon Mar 28 09:30:14 2022

@author: ctn
"""

import IntelCamera
import cv2
import KinovaArm
import utilities
import Detection

width = 640
height = 480

cam = IntelCamera.Camera(width, height) #making camera object
net = Detection.Network() #making YOLO net object

with utilities.DeviceConnection.createTcpConnection(KinovaArm.get_args()) as router:
    arm = KinovaArm.Arm(router) #create arm object
    arm.home() #move arm to home 
    while True:
        d_frame, c_img = cam.get_frames() #gets frames from camera, d_frame is of type depth_frame and c_img is a numpy array
        result = net.run_inference(c_img) #runs YOLO
        
        #do stuff here

        #shows images
        cv2.imshow('RGB', c_img)
        cv2.imshow('Depth', cam.get_depth_color(d_frame))
        
        #loop break code
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

cam.stop_stream()

print('done')