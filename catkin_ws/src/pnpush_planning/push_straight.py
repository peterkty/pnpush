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
    
    PtList = []
    
    # set of points to travel through
    PtList.append([360, -170, 140, 0, 0.7071, 0.7071, 0])
    PtList.append([360,  170, 140, 0, 0.7071, 0.7071, 0])
    PtList.append([360,    0, 140, 0, 0.7071, 0.7071, 0])
    
    # set initial configuration of the robot
    setJoint(0,0,0,0,90,90)
    
    setCart(PtList[0][0],PtList[0][1],PtList[0][2],PtList[0][3],PtList[0][4],PtList[0][5],PtList[0][6])
    raw_input('[User Input to Move] - Hit Enter after placing the block to push it: ')
    setCart(PtList[1][0],PtList[1][1],PtList[1][2],PtList[1][3],PtList[1][4],PtList[1][5],PtList[1][6])
    setCart(PtList[2][0],PtList[2][1],PtList[2][2],PtList[2][3],PtList[2][4],PtList[2][5],PtList[2][6])
    
    # set final configuration of the robot
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
