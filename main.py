# -*- coding: utf-8 -*-
import IntelCamera
import cv2
import KinovaArm
import utilities
import Detection
import numpy as np
import time
import ModelEstimator
import Controller
import Projection

width = 640
height = 480

Kp_ang = 1.5
Kp_lin = 1
max_ang_vel = 100
max_lin_vel = 0.05

minconf = 0.8

Ts = 0.22

lin_speed_x = 0
lin_speed_y = 0
lin_speed_z = 0
center = (0,0)
X = [0,0,0]

started = False
detected = False

cam = IntelCamera.Camera(width, height) #making camera object
net = Detection.Network() #making YOLO net object
mod_est = ModelEstimator.ModelEstimator(Ts) #model based estimator
control = Controller.Controller(Kp_lin, Kp_ang, max_ang_vel, max_lin_vel)
#proj = Projection.Projection(cam, 0.083, -0.08, 25, width, height)
proj = Projection.Projection(cam, 0.083, -0.05, 0, width, height)
#proj = Projection.Projection(cam, 0, 0, 0, width, height)

#gripper_coords = proj.get_gripper_proj()

with utilities.DeviceConnection.createTcpConnection(KinovaArm.get_args()) as router:
    arm = KinovaArm.Arm(router) #create arm object
    arm.home() #move arm to home 
    arm.gripper(0)
    
    while True:
        t_start = time.time()
        d_frame, c_img = cam.get_frames()
        #cv2.circle(c_img, (int(gripper_coords[0]),int(gripper_coords[1])), 5, (0,0,255), 2)
        
        if started:
            result = net.run_inference(c_img)
            p1, p2, conf = net.get_bottle(result)
            
            if(p1 != (-1,-1) and p2 != (-1,-1) and conf > minconf):
                center = net.get_middle(p1,p2)
                cv2.circle(c_img, center, 5, (0,255,0), 2)
                cv2.rectangle(c_img, p1, p2, (0,255,0), 2)
                d = cam.get_dist(d_frame, center[0], center[1])
                X = proj.get3d_point(center, d)
    
                data = arm.get_data()
                vx = data.tool_twist_linear_x
                vy = data.tool_twist_linear_y
                vz = data.tool_twist_linear_z
                e_theta_x = 0#(90)-data.tool_pose_theta_x
                
                lin_speed_x, lin_speed_y, lin_speed_z, ang_speed_x, ang_speed_y = control.get_control(X, e_theta_x)

                mod_est.update(X, np.array([[vx], [vy], [vz]]), np.array([[lin_speed_x], [lin_speed_y], [lin_speed_z]]))
                arm.send_speeds(lin_speed_x, lin_speed_y, lin_speed_z, ang_speed_x, ang_speed_y, 0, 0)
                last_t = time.time()
                detected = True
                print(str(float(X[0])), str(float(X[1])), str(float(X[2])), conf, sep='\t')
                
            elif(detected):
                data = arm.get_data()
                vx = data.tool_twist_linear_x
                vy = data.tool_twist_linear_y
                vz = data.tool_twist_linear_z
                
                X = mod_est.estimate(np.array([[lin_speed_x], [lin_speed_y], [lin_speed_z]]))
                print('e')
                print(str(float(X[0])), str(float(X[1])), str(float(X[2])), sep='\t')
                if(np.linalg.norm(X) < 0.00001):
                    arm.send_speeds(0, 0, 0, 0, 0, 0, 0)
                    #arm.gripper(1)
                    time.sleep(5)
                    data = arm.get_data()
                    print(data.tool_pose_x,data.tool_pose_y,data.tool_pose_z, sep='\t')
                    arm.send_pose(data.tool_pose_x, data.tool_pose_y, data.tool_pose_z+0.1, data.tool_pose_theta_x, data.tool_pose_theta_y, data.tool_pose_theta_z)
                    time.sleep(10)
                    data = arm.get_data()
                    print(data.tool_pose_x,data.tool_pose_y,data.tool_pose_z, sep='\t')
                    arm.home()
                    break
                
                lin_speed_x, lin_speed_y, lin_speed_z, ang_speed_x, ang_speed_y = control.get_control(X, e_theta_x)
                est_dot_x, est_dot_y = proj.get_proj(X)
                cv2.circle(c_img, (est_dot_x, est_dot_y), 5, (252,240,3), 2)
                arm.send_speeds(lin_speed_x, lin_speed_y, lin_speed_z, 0, 0, 0, 0)
                last_t = time.time()
                
        #shows images
        cv2.imshow('RGB', c_img)
        started = True
        #cv2.imshow('Depth', cam.get_depth_color(d_frame))
        
        #loop break code
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

            
cam.stop_stream()
print('done')           
