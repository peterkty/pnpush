__author__ = 'nimafazeli'

import geometry_msgs
import std_msgs
import json
from ik.ik import IK
import tf.transformations as tfm
from ik.roshelper import *
from ik.helper import *
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
import roslib; 
roslib.load_manifest("netft_rdt_driver")
from netft_rdt_driver.srv import Zero
from config.shape_db_moreshape import ShapeDB
import subprocess, os, signal

def wait_for_ft_calib():
    ROS_Wait_For_Msg('/netft_data', geometry_msgs.msg.WrenchStamped).getmsg()

setCartRos = rospy.ServiceProxy('/robot2_SetCartesian', robot_SetCartesian)
addBufferRos = rospy.ServiceProxy('/robot2_AddBuffer', robot_AddBuffer)
def setCart(pos, ori):
    param = (np.array(pos) * 1000).tolist() + ori
    setCartRos(*param)
    
def addBuffer(pos, ori):
    param = (np.array(pos) * 1000).tolist() + ori
    addBufferRos(*param)

def push_straight():

    rospy.init_node('listener', anonymous=True)
    listener = tf.TransformListener()
    
    rospy.sleep(1)
    l2 = 0 # to be changed based on length of pusher
    tip_hand_transform = [0,0,l2,0,0,0] # x,y,z,roll,pitch,yaw

    # define set cartesian, get cartesian and set joints
    #setCart = rospy.ServiceProxy('/robot2_SetCartesian', robot_SetCartesian)
    #getCart = rospy.ServiceProxy('/robot2_GetCartesian', robot_GetCartesian)
    setJoint = rospy.ServiceProxy('/robot2_SetJoints', robot_SetJoints)
    setSpeed = rospy.ServiceProxy('/robot2_SetSpeed', robot_SetSpeed)
    #addBuffer = rospy.ServiceProxy('/robot2_AddBuffer', robot_AddBuffer)
    clearBuffer = rospy.ServiceProxy('/robot2_ClearBuffer', robot_ClearBuffer)
    executeBuffer = rospy.ServiceProxy('/robot2_ExecuteBuffer', robot_ExecuteBuffer)
    setZero = rospy.ServiceProxy('/zero', Zero)
    setZone = rospy.ServiceProxy('/robot2_SetZone', robot_SetZone)
    
    # parameters about rosbag
    dir_save_bagfile = os.environ['PNPUSHDATA_BASE'] + '/push_dataset_id/'
    #topics = ['/joint_states', '/netft_data', '/tf', '/visualization_marker']
    topics = ['-a']
    
    global_speed = 200
    setSpeed(global_speed,1000)
    surface_thick = 0.01158   # 0.01158 for plywood
    z = 0.217+surface_thick # long 279 || short 217
    zup = 0.217+surface_thick+0.040 
    ori = [0,0,1,0]
    numOfRuns = 10
    top_speeds = np.linspace(500, 1000.0, numOfRuns) 
    
    # set initial configuration of the robot
    setJoint(0,0,0,0,90,0)
    
    # parameters about object
    shape_id = 'rect1'
    shape_db = ShapeDB()
    shape_type = shape_db.shape_db[shape_id]['shape_type']
    shape = shape_db.shape_db[shape_id]['shape']
    obj_frame_id = shape_db.shape_db[shape_id]['frame_id']
    
    dist_before_contact = 0.005
    dist_after_contact = 0.550 
    
    global_frame_id = '/map'
    
    probe_radius = 0.004745    # probe1: 0.00626/2 probe2: 0.004745
    
    for counter in range(0, numOfRuns):
        top_speed = top_speeds[counter]
        bagfilename = dir_save_bagfile+('%d.bag' % counter)
        print 'Run number %s' % counter
        
        # find the probe pos in contact in object frame
        pos = np.array(shape[2]) * 0.5  + np.array(shape[(3)]) *(0.5) 
        pos = np.append(pos, [0])
        
        tangent = np.array(shape[(3)]) - np.array(shape[2])
        normal = np.array([tangent[1], -tangent[0]]) 
        normal = normal / norm(normal)  # normalize it
        normal = np.append(normal, [0])
                    
        pos_probe_contact_object = pos + normal * probe_radius
        # find the start point
        direc =  normal
        pos_start_probe_object = pos_probe_contact_object + direc * dist_before_contact
        # find the end point
        pos_end_probe_object = pos_probe_contact_object - direc * dist_after_contact
        
        # transform start and end to world frame
        pos_start_probe_world = coordinateFrameTransform(pos_start_probe_object, obj_frame_id, global_frame_id, listener)
        pos_end_probe_world = coordinateFrameTransform(pos_end_probe_object, obj_frame_id, global_frame_id, listener)
        
        PtList = []
        N = 100
        for i in range(0,N):
            s = float(i) / N
            PtList.append((np.array(pos_start_probe_world) * (1-s) + np.array(pos_end_probe_world) * (s)).tolist())
            
        setCart([PtList[0][0],PtList[0][1],zup], ori)
        setCart([PtList[0][0],PtList[0][1],z], ori)
        
        # zero force torque sensor
        rospy.sleep(1)
        setZero()
        wait_for_ft_calib()
        
        
        # begin recording
        rosbag_proc = subprocess.Popen('rosbag record -q -O %s %s' % (bagfilename, " ".join(topics)) , shell=True, cwd=dir_save_bagfile)
        print 'rosbag_proc.pid=', rosbag_proc.pid
        rospy.sleep(0.5)
        
        
        clearBuffer()
        setZone(5)
        for i in range(0,N):
            setSpeed((top_speed-50.0)*float(i)/N+50,2000)
            #print (top_speed-50.0)*float(i)/N+50
            addBuffer([PtList[i][0],PtList[i][1],z],ori)
        executeBuffer()
        setZone(1)
        
        rospy.sleep(1)
        # end bag recording
        terminate_ros_node("/record")
    
        # set final configuration of the robot
        setSpeed(global_speed,2000)
        setJoint(0,0,0,0,90,0)
        raw_input('[User Input to Move] - Hit Enter after placing the block to push it: ')
    
    print('[Execution] - End of program!')


def terminate_ros_node(s):
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for str in list_output.split("\n"):
        if (str.startswith(s)):
            os.system("rosnode kill " + str)
            
if __name__=='__main__':
    global listener
    
    # forward backward test
    push_straight()
