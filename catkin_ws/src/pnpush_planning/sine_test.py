#!/usr/bin/env python

import geometry_msgs
import std_msgs
import json
from ik.roshelper import ROS_Wait_For_Msg
from ik.ik import IK
import tf.transformations as tfm
import rospy
import pdb
import sys
import tf
import math
import numpy as np
import tf.transformations as tfm
from robot_comm.srv import *

def sine_test(beginPt = [374, 0, 400], endPt = [474, 0, 400]):
    
    l2 = 0  # check testik for value
    tip_hand_transform = [0, 0, l2, 0,0,0] # x,y,z,r,p,y
    joint_topic = '/joint_states'
    
    listener = tf.TransformListener()
    rospy.sleep(0.1)    
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)
    
    plans = []
    
    beginPt.extend([0,0.7071,0.7071,0])
    endPt.extend([0,0.7071,0.7071,0])
    
    print(beginPt)
    
    #zero point
    # planner = IK(target_tip_pos = beginPt[0:3], target_tip_ori = beginPt[3:7], 
             # joint_topic=joint_topic, tip_hand_transform=tip_hand_transform, 
             # straightness = 0) 
    # plan0 = planner.plan()
    # plan0.setSpeedByName('fast')
    # 
    # plans.append(plan0)
    # plan0.execute()
    # 
    # #first point
    # planner = IK(target_tip_pos = endPt[0:3], target_tip_ori = endPt[3:7], 
             # joint_topic=joint_topic, tip_hand_transform=tip_hand_transform, 
             # straightness = 0.95) 
    # plan1 = planner.plan()
    # plan1.setSpeedByName('faster')
# 
    # plans.append(plan1)
    # 
    # #second point
    # planner = IK(target_tip_pos = beginPt[0:3], target_tip_ori = beginPt[3:7], 
             # joint_topic=joint_topic, tip_hand_transform=tip_hand_transform, 
             # straightness = 0.95) 
    # plan2 = planner.plan()
    # plan2.setSpeedByName('faster')
    # plans.append(plan2)
    # 
    # for x in range(0,2):
        # plans.append(plan1)
        # plans.append(plan2)
    
    ## visualize the frame
    br.sendTransform(tuple(tip_hand_transform[0:3]), 
                    tfm.quaternion_from_euler(*tip_hand_transform[3:6]), 
                    rospy.Time.now(), 'tip', "link_6")
    
    # for numOfPlan in range(0, len(plans)):
        # plans[numOfPlan].visualize()
        # raw_input('Execute? ')
        # plans[numOfPlan].execute()
    
    setCart = rospy.ServiceProxy('/robot2_SetCartesian', robot_SetCartesian)
    setJoint = rospy.ServiceProxy('/robot2_SetJoints', robot_SetJoints)
    
    setJoint(0,0,0,0,90,90)
    
    for x in range(0,9):
        setCart(beginPt[0],beginPt[1],beginPt[2],beginPt[3],beginPt[4],beginPt[5],beginPt[6])
        setCart(endPt[0],endPt[1],endPt[2],endPt[3],endPt[4],endPt[5],endPt[6])
    

if __name__=='__main__':
    # forward backward test
    rospy.init_node('listener', anonymous=True)
    
    sine_test()
