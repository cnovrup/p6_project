# -*- coding: utf-8 -*-
"""
Created on Tue Mar 29 10:15:43 2022

@author: ctn
"""

import sys
import os
import time
import threading
import numpy as np
import math

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

def get_args():
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities
    args = utilities.parseConnectionArguments()
    return args

class Arm:
    def __init__(self, router):
        self.router = router
        self.base = BaseClient(self.router)
        self.base_cyclic = BaseCyclicClient(self.router)
        self.TIMEOUT_DURATION = 20
        
    def check_for_end_or_abort(self,e):
        """Return a closure checking for END or ABORT notifications

        Arguments:
        e -- event to signal when the action is completed
            (will be set when an END or ABORT occurs)
        """
        def check(notification, e = e):
            print("EVENT : " + \
                  Base_pb2.ActionEvent.Name(notification.action_event))
            if notification.action_event == Base_pb2.ACTION_END \
            or notification.action_event == Base_pb2.ACTION_ABORT:
                e.set()
        return check
    
    def home(self):
        # Make sure the arm is in math.single Level Servoing mode
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(base_servo_mode)
        
        # Move arm to ready position
        print("Moving the arm to a safe position")
        action_type = Base_pb2.RequestedActionType()
        action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
        action_list = self.base.ReadAllActions(action_type)
        action_handle = None
        for action in action_list.action_list:
            if action.name == "testhome2":
                action_handle = action.handle

        if action_handle == None:
            print("Can't reach safe position. Exiting")
            return False

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        self.base.ExecuteActionFromReference(action_handle)
        finished = e.wait(self.TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)

        if finished:
            print("Safe position reached")
        else:
            print("Timeout on action notification wait")
        return finished
    
    def send_speeds(self, x, y, z, th_x, th_y, th_z, duration):
        command = Base_pb2.TwistCommand()
        command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
        command.duration = duration
        
        twist = command.twist
        twist.linear_x = x
        twist.linear_y = y
        twist.linear_z = z
        twist.angular_x = th_x
        twist.angular_y = th_y
        twist.angular_z = th_z
        
        self.base.SendTwistCommand(command)
        
        # Let time for twist to be executed
        #time.sleep(5)
        
        #self.base.Stop()
        #time.sleep(1)
        
        return True
    
    def get_data(self):
        feedback = self.base_cyclic.RefreshFeedback()
        return feedback.base

    def get_joint_angles(self):
        jointangles = self.base.GetMeasuredJointAngles()
        jointangles_list = [0,0,0,0,0,0]
        for i in range(6):
            jointangle = jointangles.joint_angles[i]
            jointangles_list[jointangle.joint_identifier] = jointangle.value
        return jointangles_list

    def get_joint_angles_rad(self):
        jointangles = self.base.GetMeasuredJointAngles()
        jointangles_list = [0,0,0,0,0,0]
        for i in range(6):
            jointangle = jointangles.joint_angles[i]
            jointangles_list[jointangle.joint_identifier] = math.radians(float(jointangle.value))
        return jointangles_list    

    def get_pose(self):
        return self.base.GetMeasuredCartesianPose()

    def send_pose(self, x, y, z, th_x, th_y, th_z):
        action = Base_pb2.Action()

        cartesian_pose = action.reach_pose.target_pose
        cartesian_pose.x = x    # (meters)
        cartesian_pose.y = y     # (meters)
        cartesian_pose.z = z     # (meters)
        cartesian_pose.theta_x = th_x # (degrees)
        cartesian_pose.theta_y = th_y # (degrees)
        cartesian_pose.theta_z = th_z # (degrees)

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        self.base.ExecuteAction(action)

        finished = e.wait(self.TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)

    def send_jointangles(self, joint_angle_list):
        action = Base_pb2.Action()

        for joint_id in range(6):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = joint_id
            joint_angle.value = joint_angle_list[joint_id]

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions())
            
        self.base.ExecuteAction(action)

        finished = e.wait(self.TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)

    def gripper(self, opening):
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        finger.finger_identifier = 1
        finger.value = opening
        self.base.SendGripperCommand(gripper_command)
        
    def get_trans_gripper(self):
        q = self.get_joint_angles_rad()
        T = np.array([[  math.sin(q[5] + math.pi/2)*(math.cos(q[3] + math.pi/2)*math.sin(q[0]) + math.sin(q[3] + math.pi/2)*(math.cos(q[0])*math.sin(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2) + math.cos(q[0])*math.cos(q[1] + math.pi/2)*math.cos(q[2] + math.pi/2))) + math.cos(q[5] + math.pi/2)*(math.cos(q[4])*(math.sin(q[0])*math.sin(q[3] + math.pi/2) - math.cos(q[3] + math.pi/2)*(math.cos(q[0])*math.sin(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2) + math.cos(q[0])*math.cos(q[1] + math.pi/2)*math.cos(q[2] + math.pi/2))) - math.sin(q[4])*(math.cos(q[0])*math.cos(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2) - math.cos(q[0])*math.cos(q[2] + math.pi/2)*math.sin(q[1] + math.pi/2))), math.cos(q[5] + math.pi/2)*(math.cos(q[3] + math.pi/2)*math.sin(q[0]) + math.sin(q[3] + math.pi/2)*(math.cos(q[0])*math.sin(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2) + math.cos(q[0])*math.cos(q[1] + math.pi/2)*math.cos(q[2] + math.pi/2))) - math.sin(q[5] + math.pi/2)*(math.cos(q[4])*(math.sin(q[0])*math.sin(q[3] + math.pi/2) - math.cos(q[3] + math.pi/2)*(math.cos(q[0])*math.sin(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2) + math.cos(q[0])*math.cos(q[1] + math.pi/2)*math.cos(q[2] + math.pi/2))) - math.sin(q[4])*(math.cos(q[0])*math.cos(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2) - math.cos(q[0])*math.cos(q[2] + math.pi/2)*math.sin(q[1] + math.pi/2))), math.sin(q[4])*(math.sin(q[0])*math.sin(q[3] + math.pi/2) - math.cos(q[3] + math.pi/2)*(math.cos(q[0])*math.sin(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2) + math.cos(q[0])*math.cos(q[1] + math.pi/2)*math.cos(q[2] + math.pi/2))) + math.cos(q[4])*(math.cos(q[0])*math.cos(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2) - math.cos(q[0])*math.cos(q[2] + math.pi/2)*math.sin(q[1] + math.pi/2)), 10*math.sin(q[0]) + 280*math.cos(q[0])*math.cos(q[1] + math.pi/2) + 57*math.cos(q[3] + math.pi/2)*math.sin(q[0]) + 235*math.sin(q[4])*(math.sin(q[0])*math.sin(q[3] + math.pi/2) - math.cos(q[3] + math.pi/2)*(math.cos(q[0])*math.sin(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2) + math.cos(q[0])*math.cos(q[1] + math.pi/2)*math.cos(q[2] + math.pi/2))) + 235*math.cos(q[4])*(math.cos(q[0])*math.cos(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2) - math.cos(q[0])*math.cos(q[2] + math.pi/2)*math.sin(q[1] + math.pi/2)) + 57*math.sin(q[3] + math.pi/2)*(math.cos(q[0])*math.sin(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2) + math.cos(q[0])*math.cos(q[1] + math.pi/2)*math.cos(q[2] + math.pi/2)) + 245*math.cos(q[0])*math.cos(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2) - 245*math.cos(q[0])*math.cos(q[2] + math.pi/2)*math.sin(q[1] + math.pi/2)],
                      [- math.cos(q[5] + math.pi/2)*(math.cos(q[4])*(math.cos(q[0])*math.sin(q[3] + math.pi/2) + math.cos(q[3] + math.pi/2)*(math.cos(q[1] + math.pi/2)*math.cos(q[2] + math.pi/2)*math.sin(q[0]) + math.sin(q[0])*math.sin(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2))) + math.sin(q[4])*(math.cos(q[1] + math.pi/2)*math.sin(q[0])*math.sin(q[2] + math.pi/2) - math.cos(q[2] + math.pi/2)*math.sin(q[0])*math.sin(q[1] + math.pi/2))) - math.sin(q[5] + math.pi/2)*(math.cos(q[0])*math.cos(q[3] + math.pi/2) - math.sin(q[3] + math.pi/2)*(math.cos(q[1] + math.pi/2)*math.cos(q[2] + math.pi/2)*math.sin(q[0]) + math.sin(q[0])*math.sin(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2))), math.sin(q[5] + math.pi/2)*(math.cos(q[4])*(math.cos(q[0])*math.sin(q[3] + math.pi/2) + math.cos(q[3] + math.pi/2)*(math.cos(q[1] + math.pi/2)*math.cos(q[2] + math.pi/2)*math.sin(q[0]) + math.sin(q[0])*math.sin(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2))) + math.sin(q[4])*(math.cos(q[1] + math.pi/2)*math.sin(q[0])*math.sin(q[2] + math.pi/2) - math.cos(q[2] + math.pi/2)*math.sin(q[0])*math.sin(q[1] + math.pi/2))) - math.cos(q[5] + math.pi/2)*(math.cos(q[0])*math.cos(q[3] + math.pi/2) - math.sin(q[3] + math.pi/2)*(math.cos(q[1] + math.pi/2)*math.cos(q[2] + math.pi/2)*math.sin(q[0]) + math.sin(q[0])*math.sin(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2))), math.cos(q[4])*(math.cos(q[1] + math.pi/2)*math.sin(q[0])*math.sin(q[2] + math.pi/2) - math.cos(q[2] + math.pi/2)*math.sin(q[0])*math.sin(q[1] + math.pi/2)) - math.sin(q[4])*(math.cos(q[0])*math.sin(q[3] + math.pi/2) + math.cos(q[3] + math.pi/2)*(math.cos(q[1] + math.pi/2)*math.cos(q[2] + math.pi/2)*math.sin(q[0]) + math.sin(q[0])*math.sin(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2))), 280*math.cos(q[1] + math.pi/2)*math.sin(q[0]) - 57*math.cos(q[0])*math.cos(q[3] + math.pi/2) - 10*math.cos(q[0]) - 235*math.sin(q[4])*(math.cos(q[0])*math.sin(q[3] + math.pi/2) + math.cos(q[3] + math.pi/2)*(math.cos(q[1] + math.pi/2)*math.cos(q[2] + math.pi/2)*math.sin(q[0]) + math.sin(q[0])*math.sin(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2))) + 235*math.cos(q[4])*(math.cos(q[1] + math.pi/2)*math.sin(q[0])*math.sin(q[2] + math.pi/2) - math.cos(q[2] + math.pi/2)*math.sin(q[0])*math.sin(q[1] + math.pi/2)) + 57*math.sin(q[3] + math.pi/2)*(math.cos(q[1] + math.pi/2)*math.cos(q[2] + math.pi/2)*math.sin(q[0]) + math.sin(q[0])*math.sin(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2)) + 245*math.cos(q[1] + math.pi/2)*math.sin(q[0])*math.sin(q[2] + math.pi/2) - 245*math.cos(q[2] + math.pi/2)*math.sin(q[0])*math.sin(q[1] + math.pi/2)],
                      [                                                                                                      - math.cos(q[5] + math.pi/2)*(math.sin(q[4])*(math.cos(q[1] + math.pi/2)*math.cos(q[2] + math.pi/2) + math.sin(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2)) - math.cos(q[4])*math.cos(q[3] + math.pi/2)*(math.cos(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2) - math.cos(q[2] + math.pi/2)*math.sin(q[1] + math.pi/2))) - math.sin(q[3] + math.pi/2)*math.sin(q[5] + math.pi/2)*(math.cos(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2) - math.cos(q[2] + math.pi/2)*math.sin(q[1] + math.pi/2)),                                                                                                       math.sin(q[5] + math.pi/2)*(math.sin(q[4])*(math.cos(q[1] + math.pi/2)*math.cos(q[2] + math.pi/2) + math.sin(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2)) - math.cos(q[4])*math.cos(q[3] + math.pi/2)*(math.cos(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2) - math.cos(q[2] + math.pi/2)*math.sin(q[1] + math.pi/2))) - math.cos(q[5] + math.pi/2)*math.sin(q[3] + math.pi/2)*(math.cos(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2) - math.cos(q[2] + math.pi/2)*math.sin(q[1] + math.pi/2)),                                                            math.cos(q[4])*(math.cos(q[1] + math.pi/2)*math.cos(q[2] + math.pi/2) + math.sin(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2)) + math.cos(q[3] + math.pi/2)*math.sin(q[4])*(math.cos(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2) - math.cos(q[2] + math.pi/2)*math.sin(q[1] + math.pi/2)),                                                                                                                                   280*math.sin(q[1] + math.pi/2) + 235*math.cos(q[4])*(math.cos(q[1] + math.pi/2)*math.cos(q[2] + math.pi/2) + math.sin(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2)) - 57*math.sin(q[3] + math.pi/2)*(math.cos(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2) - math.cos(q[2] + math.pi/2)*math.sin(q[1] + math.pi/2)) + 245*math.cos(q[1] + math.pi/2)*math.cos(q[2] + math.pi/2) + 245*math.sin(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2) + 235*math.cos(q[3] + math.pi/2)*math.sin(q[4])*(math.cos(q[1] + math.pi/2)*math.sin(q[2] + math.pi/2) - math.cos(q[2] + math.pi/2)*math.sin(q[1] + math.pi/2)) + 2433/10],
                      [                                                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                                                                                                                                                                                                                                                                                                       0,                                                                                                                                                                                                                           0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     1]])
        return T[0:3, 0:3], T[0:3,3:4]
    
