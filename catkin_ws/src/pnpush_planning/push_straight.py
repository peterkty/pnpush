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

def push_straight():

    global listener
    l2 = 0 # to be changed based on length of pusher
    tip_hand_transform = [0,0,l2,0,0,0] # x,y,z,roll,pitch,yaw

    # define set cartesian, get cartesian and set joints
    setCart = rospy.ServiceProxy('/robot2_SetCartesian', robot_SetCartesian)
    getCart = rospy.ServiceProxy('/robot2_GetCartesian', robot_GetCartesian)
    setJoint = rospy.ServiceProxy('/robot2_SetJoints', robot_SetJoints)
