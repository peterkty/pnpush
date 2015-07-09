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
from sensor_msgs.msg import *
from geometry_msgs.msg import PoseStamped
import geometry_msgs.msg

def sine_test(beginPt = [374, 0, 400], endPt = [474, 0, 400]):
    global listener
    l2 = 0  # check testik for value
    tip_hand_transform = [0, 0, l2, 0,0,0] # x,y,z,r,p,y
    joint_topic = '/joint_states'
    
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)
    
    plans = []
    beginPt.extend([0,0.7071,0.7071,0])
    endPt.extend([0,0.7071,0.7071,0])
    
    ## visualize the frame
    br.sendTransform(tuple(tip_hand_transform[0:3]), 
                    tfm.quaternion_from_euler(*tip_hand_transform[3:6]), 
                    rospy.Time.now(), 'tip', "link_6")
    
    setCart = rospy.ServiceProxy('/robot2_SetCartesian', robot_SetCartesian)
    setJoint = rospy.ServiceProxy('/robot2_SetJoints', robot_SetJoints)
    
    setJoint(0,0,0,0,90,90)
    
    for x in range(0,2):
        setCart(beginPt[0],beginPt[1],beginPt[2],beginPt[3],beginPt[4],beginPt[5],beginPt[6])
        setCart(endPt[0],endPt[1],endPt[2],endPt[3],endPt[4],endPt[5],endPt[6])
        
def callback(data):
    global pub
    global listener
    # data.data.position[1] = data.data.position[1] * 100
    # pub.publish(data)
    t=rospy.Time.now()
    print(data)
    # print t
    pose = listener.lookupTransform('link_ft','base_link',t)
    pub.publish(pose)


if __name__=='__main__':
    global pub
    global listener
    
    # forward backward test
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, callback)
    pub = rospy.Publisher('/tip_pose', PoseStamped, queue_size=1)
    listener = tf.TransformListener()
    rospy.sleep(0.1)
    sine_test()
