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
    setSpeed = rospy.ServiceProxy('/robot2_SetSpeed', robot_SetSpeed)
    
    setSpeed(300,50)
    z = 279
    beginTravel = [360, -300, z]
    endTravel   = [360,  250, z]
    
    PtList = []
    
    delta = endTravel[1] - beginTravel[1]
    N = 100
    for i in range(0,N):
        PtList.append([360,  beginTravel[1]+ (delta/N)*i, z, 0, 0.7071, 0.7071, 0])
    
    # set initial configuration of the robot
    setJoint(0,0,0,0,90,90)
    
    setCart(PtList[0][0],PtList[0][1],PtList[0][2],PtList[0][3],PtList[0][4],PtList[0][5],PtList[0][6])
    raw_input('[User Input to Move] - Hit Enter after placing the block to push it: ')
    
    for i in range(0,N):
        setCart(PtList[i][0],PtList[i][1],PtList[i][2],PtList[i][3],PtList[i][4],PtList[i][5],PtList[i][6])
        setSpeed((500-50)*i/N+50,50)
    
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
