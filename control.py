import IntelCamera
import cv2
import KinovaArm
import utilities
import Detection
import numpy as np
from custom_utils import saturation

width = 640
height = 480


cam = IntelCamera.Camera(width, height) #making camera object
net = Detection.Network() #making YOLO net object
started = False


with utilities.DeviceConnection.createTcpConnection(KinovaArm.get_args()) as router:
    arm = KinovaArm.Arm(router) #create arm object
    arm.home() #move arm to home 
    print(cam.get_intr())
    
    R = np.array([[1,0,0],[0,-1,0],[0,0,1]])
    K = np.array([[cam.fx,0,cam.u0],[0, cam.fy, -cam.v0],[0,0,1]]);
    t = np.array([[0],[-0.070],[0]])
    #t = np.array([[0],[0],[0]])
    KR = K @ R
    M = np.c_[KR,t];
    KRinv = np.linalg.inv(KR)
    d = -1
    
    while True:
        d_frame, c_img = cam.get_frames() #gets frames from camera, d_frame is of type depth_frame and c_img is a numpy array
        result = net.run_inference(c_img) #runs YOLO
        cv2.circle(c_img, (int(cam.u0),int(cam.v0)), 5, (0,0,255), 2)
        #do stuff here

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
                           
                Kp_ang = 20
                Kp_lin = 1
                max_ang_vel = 30
                max_lin_vel = 0.05
                
                feedback = arm.get_data()
                e_theta_x = (90)-feedback.tool_pose_theta_x
                
                lin_speed_x = -Kp_lin*saturation(float(X[0]), max_lin_vel, -max_lin_vel)
                lin_speed_y = -Kp_lin*saturation(float(X[1]), max_lin_vel, -max_lin_vel)
                lin_speed_z = Kp_lin*saturation(float(X[2]), max_lin_vel, -max_lin_vel)
                ang_speed_x = saturation(e_theta_x, max_ang_vel, -max_ang_vel)
                ang_speed_y = -Kp_ang*saturation(float(X[0]), max_ang_vel, -max_ang_vel)
                
                #print(lin_speed_x, lin_speed_y, lin_speed_z, ang_speed_x, ang_speed_y, sep='\t')
                print(X[0], X[1], X[2], sep='\t')
                #print(d)
                arm.send_speeds(lin_speed_x, lin_speed_y, lin_speed_z, ang_speed_x, ang_speed_y, 0, 0)
            #arm.send_speeds(0.1, 0, 0, 0, 0, 0, 0)
            
        except Exception as e:
            print(e)
            #if d > 0 and d < 0.15:
            arm.send_speeds(0, 0, 0, 0, 0, 0, 0)

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

cam.stop_stream()

print('done')
