import IntelCamera
import cv2
import KinovaArm
import utilities
import Detection
import numpy as np
from custom_utils import saturation
import time
import PositionEstimator

width = 640
height = 480


cam = IntelCamera.Camera(width, height) #making camera object
net = Detection.Network() #making YOLO net object
pos_est = PositionEstimator.PositionEstimator(np.array([[0],[0],[0]]))

                
Kp_ang = 0
Kp_lin = 1
max_ang_vel = 20
max_lin_vel = 0.05

started = False
detected = False


log = open('data.txt', 'w')

with utilities.DeviceConnection.createTcpConnection(KinovaArm.get_args()) as router:
    arm = KinovaArm.Arm(router) #create arm object
    arm.home() #move arm to home 
    arm.gripper(0)

    
    R = np.array([[1,0,0],[0,-1,0],[0,0,1]])
    K = np.array([[cam.fx,0,cam.u0],[0, cam.fy, -cam.v0],[0,0,1]]);
    t = np.array([[0],[-0.070],[-0.108]])
    #t = np.array([[0],[0],[0]])
    KR = K @ R
    M = np.c_[KR,t];
    KRinv = np.linalg.inv(KR)
    d = -1
    
    lin_speed_x = 0
    lin_speed_y = 0
    lin_speed_z = 0
    #X = np.array([[745], [38], [242]])
    #pos_est.setX(X)
    while True:
        t_start = time.time()
        d_frame, c_img = cam.get_frames() #gets frames from camera, d_frame is of type depth_frame and c_img is a numpy array
        '''
        if started:
            t_start = time.time()
            R1, t1 = arm.get_trans_gripper()
            X = (pos_est.estimate(R1, t1))*0.001
            
            feedback = arm.get_data()
            e_theta_x = (90)-feedback.tool_pose_theta_x
            
            lin_speed_x = saturation(-Kp_lin*float(X[0]), max_lin_vel, -max_lin_vel)
            lin_speed_y = saturation(-Kp_lin*float(X[1]), max_lin_vel, -max_lin_vel)
            lin_speed_z = saturation(Kp_lin*float(X[2]), max_lin_vel, -max_lin_vel)
            ang_speed_x = saturation(e_theta_x, max_ang_vel, -max_ang_vel)
            ang_speed_y = saturation(-Kp_ang*float(X[0]), max_ang_vel, -max_ang_vel)
            
            print(lin_speed_x, lin_speed_y, lin_speed_z, ang_speed_x, ang_speed_y, sep='\t')
            #print(X[0], X[1], X[2], sep='\t')
            #print(d)
            arm.send_speeds(lin_speed_x, lin_speed_y, lin_speed_z, ang_speed_x, ang_speed_y, 0, 0)

        '''
        
        
        result = net.run_inference(c_img) #runs YOLO
        cv2.circle(c_img, (int(cam.u0),int(cam.v0)), 5, (0,0,255), 2)
        #do stuff here
        
        center = (0,0)
        X = [0,0,0]
        try:
            if started:
                p1, p2 = net.get_bottle(result)
                center = net.get_middle(p1,p2)
                cv2.circle(c_img, center, 5, (0,255,0), 2)
                cv2.rectangle(c_img, p1, p2, (0,255,0), 2)
                
                X = (KRinv @ np.array([[center[0]], [-center[1]], [1]]))
                d = cam.get_dist(d_frame, center[0], center[1])
    
                l = d/np.linalg.norm(X)
                X = t+(X*l)
                R1, t1 = arm.get_trans_gripper()
                pos_est.update(X*1000, R1, t1)
                
                
                feedback = arm.get_data()
                e_theta_x = (90)-feedback.tool_pose_theta_x
                
                lin_speed_x = saturation(-Kp_lin*float(X[0]), max_lin_vel, -max_lin_vel)
                lin_speed_y = saturation(-Kp_lin*float(X[1]), max_lin_vel, -max_lin_vel)
                lin_speed_z = saturation(Kp_lin*float(X[2]), max_lin_vel, -max_lin_vel)
                ang_speed_x = saturation(e_theta_x, max_ang_vel, -max_ang_vel)
           
                ang_speed_y = saturation(-Kp_ang*float(X[0]), max_ang_vel, -max_ang_vel)
                
                #print(lin_speed_x, lin_speed_y, lin_speed_z, ang_speed_x, ang_speed_y, sep='\t')
                #print(X[0], X[1], X[2], sep='\t')
                #print(d)
                arm.send_speeds(lin_speed_x, lin_speed_y, lin_speed_z, ang_speed_x, ang_speed_y, 0, 0)
                detected = True
            #arm.send_speeds(0.1, 0, 0, 0, 0, 0, 0)
            
        except Exception as e:
            #pass
            if detected:
                R1, t1 = arm.get_trans_gripper()
                X = (pos_est.estimate(R1, t1))*0.001
                
                if(np.linalg.norm(X) < 0.03):
                    arm.send_speeds(0, 0, 0, 0, 0, 0, 0)
                    arm.gripper(1)
                    time.sleep(5)
                    arm.home()
                    break
                
                
                feedback = arm.get_data()
                e_theta_x = (90)-feedback.tool_pose_theta_x
                
                lin_speed_x = saturation(-0.25*Kp_lin*float(X[0]), max_lin_vel, -max_lin_vel)
                lin_speed_y = saturation(-0.25*Kp_lin*float(X[1]), max_lin_vel, -max_lin_vel)
                lin_speed_z = saturation(0.25*Kp_lin*float(X[2]), max_lin_vel, -max_lin_vel)
                ang_speed_x = saturation(e_theta_x, max_ang_vel, -max_ang_vel)
                ang_speed_y = saturation(-0.25*Kp_ang*float(X[0]), max_ang_vel, -max_ang_vel)
                
                #print(lin_speed_x, lin_speed_y, lin_speed_z, ang_speed_x, ang_speed_y, sep='\t')
                #print(X[0], X[1], X[2], sep='\t')
                #print(d)
                arm.send_speeds(lin_speed_x, lin_speed_y, lin_speed_z, ang_speed_x, 0, 0, 0)
            else:
                pass
            #arm.send_speeds(0, 0, 0, 0, 0, 0, 0)
            
        # armdata = arm.get_data()
        # t_now = str(time.time() - t_start)
        # cur_x = str(armdata.tool_pose_x)
        # cur_y = str(armdata.tool_pose_y)
        # cur_z = str(armdata.tool_pose_z)
        # cur_vx = str(armdata.tool_twist_linear_x)
        # cur_vy = str(armdata.tool_twist_linear_y)
        # cur_vz = str(armdata.tool_twist_linear_z)
        # out = t_now + '\t' + cur_x + '\t' + cur_y + '\t' + cur_z + '\t' + cur_vx + '\t' + cur_vy  + '\t' + cur_vz + '\t' + str(lin_speed_x) + '\t' + str(lin_speed_y) + '\t' + str(lin_speed_z) + '\t' + str(X[0]) + '\t' + str(X[1]) + '\t' +  str(X[2])  + '\t' + str(center[0])  + '\t' + str(center[1]) 
        # log.write(out + '\n')

        #shows images
        cv2.imshow('RGB', c_img)
        started = True
        #cv2.imshow('Depth', cam.get_depth_color(d_frame))
        
        #loop break code
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
        if key & 0xFF == ord('r'):
            arm.home()
        
        print(time.time()-t_start)

cam.stop_stream()

print('done')
