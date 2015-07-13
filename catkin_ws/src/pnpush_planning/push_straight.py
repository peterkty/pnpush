__author__ = 'nimafazeli'

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
from geometry_msgs.msg import WrenchStamped

def push_straight():

    global listener
    l2 = 0 # to be changed based on length of pusher
    tip_hand_transform = [0,0,l2,0,0,0] # x,y,z,roll,pitch,yaw

    # define set cartesian, get cartesian and set joints
    setCart = rospy.ServiceProxy('/robot2_SetCartesian', robot_SetCartesian)
    getCart = rospy.ServiceProxy('/robot2_GetCartesian', robot_GetCartesian)
    setJoint = rospy.ServiceProxy('/robot2_SetJoints', robot_SetJoints)
    
    beginPt = [360, 0, 140]
    midPt   = [360, -170, 140]
    endPt   = [360, -185, 140]
    beginPt.extend([0,0.7071,0.7071,0])
    midPt.extend([0,0.7071,0.7071,0])
    endPt.extend([0,0.7071,0.7071,0])
    
    setJoint(0,0,0,0,90,90)
    setCart(beginPt[0],beginPt[1],beginPt[2],beginPt[3],beginPt[4],beginPt[5],beginPt[6])
    for x in range(0,10):
        setCart(midPt[0],midPt[1],midPt[2],midPt[3],midPt[4],midPt[5],midPt[6])
        setCart(endPt[0],endPt[1],endPt[2],endPt[3],endPt[4],endPt[5],endPt[6])
        rospy.sleep(1.)
        
    setCart(midPt[0],midPt[1],midPt[2],midPt[3],midPt[4],midPt[5],midPt[6])
    setJoint(0,0,0,0,90,90)

def callback_tip(data):
    global pub
    global listener
    # t=data.header.stamp
    # t=rospy.Time.now()
    t = listener.getLatestCommonTime('link_ft','base_link')
    (pos,ori) = listener.lookupTransform('base_link','link_ft',t)
    posest = PoseStamped()
    posest.pose.position.x = pos[0]
    posest.pose.position.y = pos[1]
    posest.pose.position.z = pos[2]
    posest.pose.orientation.x = ori[0]
    posest.pose.orientation.y = ori[1]
    posest.pose.orientation.z = ori[2]
    posest.pose.orientation.w = ori[3]
    posest.header = data.header
    
    pub.publish(posest)

def callback_ft(data):
    global pub_ft
    global listener
    wrench = WrenchStamped()
    wrench.wrench.force.x = data.wrench.force.x
    wrench.wrench.force.y = data.wrench.force.y
    wrench.wrench.force.z = data.wrench.force.z
    wrench.wrench.torque.x = data.wrench.torque.x
    wrench.wrench.torque.y = data.wrench.torque.y
    wrench.wrench.torque.z = data.wrench.torque.z
    wrench.header = data.header
    
    pub_ft.publish(wrench)



if __name__=='__main__':
    global pub
    global pub_ft
    global listener
    
    # forward backward test
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, callback_tip)
    rospy.Subscriber("/netft_data", WrenchStamped, callback_ft)
    pub    = rospy.Publisher('/tip_pose', PoseStamped, queue_size=1)
    pub_ft = rospy.Publisher('/forcetorque', WrenchStamped, queue_size=1)
    listener = tf.TransformListener()
    rospy.sleep(0.1)
    push_straight()
