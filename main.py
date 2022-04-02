# -*- coding: utf-8 -*-
"""
Created on Mon Mar 28 09:30:14 2022

@author: ctn
"""

import IntelCamera
import cv2

# cam = IntelCamera.Camera()
# d_frame, c_img = cam.get_frames()
# cv2.imshow('RGB', c_img)
# cv2.imshow('Depth', cam.get_depth_color(d_frame))
# print(cam.get_dist(d_frame, int(640/2), int(480/2)))
# cv2.waitKey(0)
# cam.stop_stream()
# cv2.destroyAllWindows()

import KinovaArm
import utilities
import Detection

# with utilities.DeviceConnection.createTcpConnection(KinovaArm.get_args()) as router:
#     arm = KinovaArm.Arm(router)
#     arm.home()
#     arm.send_speeds(0, 0, 0, 0, 10, 0, 2)



width = 640
height = 480

cam = IntelCamera.Camera(width, height)
net = Detection.Network()

#with utilities.DeviceConnection.createTcpConnection(KinovaArm.get_args()) as router:
#arm = KinovaArm.Arm(router)
#arm.home()
while True:
    d_frame, c_img = cam.get_frames()
    
    result = net.run_inference(c_img)
    
    try:
        p1, p2 = net.get_bottle(result)
        x_mid, y_mid = net.get_middle(p1, p2)
        xmin = p1[0]
        xmax = p2[0]
        ymin = p1[1]
        ymax = p2[1]
        
        cv2.circle(c_img, (int(640/2), int(480/2)), 5, (0,0,255), 2)
        cv2.circle(c_img, (int(x_mid), int(y_mid)), 5, (0,255,0), 2)
        cv2.rectangle(c_img, (int(xmin),int(ymin)), (int(xmax), int(ymax)), (0,255,0), 2)
        
        rho = cam.get_dist(d_frame, int(x_mid), int(y_mid))
        
        # Kp_ang = 0.05
        # Kp_lin = 1
        # max_ang_speed = 5
        # max_lin_speed = 0.03
        # speed_th_x = Kp_ang*(y_mid-height/2)
        # speed_th_y = -Kp_ang*(x_mid-width/2)
        # speed_z = Kp_lin*rho
        
        
        # if(speed_th_x > max_ang_speed):
        #     speed_th_x = max_ang_speed
        # if(speed_th_x < -max_ang_speed):
        #     speed_th_x = -max_ang_speed
        # if(speed_th_y > max_ang_speed):
        #     speed_th_y = max_ang_speed
        # if(speed_th_y < -max_ang_speed):
        #     speed_th_y = -max_ang_speed
        # if(speed_z > max_lin_speed):
        #     speed_z = max_lin_speed
        # if(speed_z < -max_lin_speed):
        #     speed = max_lin_speed
            
        # arm.send_speeds(0, 0, speed_z, speed_th_x, speed_th_y, 0, 0)
        
        
        theta, phi = cam.get_angles(int(x_mid), int(y_mid), 640, 480)
        
        
        h_bottle = (rho*100*(ymax-ymin)*1080*1.4*0.001)/(1.88*480)
        
        
        dx, dy, dz = cam.get_cart(theta, phi, rho)
        #print(round(theta, 3), round(phi,3), round(dx,3),round(dy, 3), round(dz, 3),'\t')
        #print(x_mid, y_mid, '\t')
        print(cam.get_intr().ppx)
        
    except Exception as e:
        #arm.send_speeds(0, 0, 0, 0, 0, 0, 0)
        print(e)
        #break
        #pass
    
    cv2.imshow('RGB', c_img)
    cv2.imshow('Depth', cam.get_depth_color(d_frame))
    
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q') or key == 27:
        cv2.destroyAllWindows()
        break

cam.stop_stream()

print('done')