#!/usr/bin/env python

# Peter KT Yu, Aug 2015
# automate the process of pushing against an object and record the data

import sys
import numpy as np
from ik.roshelper import ROS_Wait_For_Msg
from ik.roshelper import lookupTransform
from ik.roshelper import coordinateFrameTransform
from ik.helper import Timer
from ik.ik import setSpeed
from geometry_msgs.msg import WrenchStamped
import tf
import tf.transformations as tfm
import rospy
import json
import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *
import sensor_msgs.msg
import geometry_msgs.msg
import os
import scipy.io as sio
from visualization_msgs.msg import Marker
from marker_helper import createMeshMarker
from marker_helper import createPointMarker
from marker_helper import createArrowMarker
from marker_helper import createSphereMarker
from tf.broadcaster import TransformBroadcaster
from math import pi
import pdb
import copy
import subprocess, os, signal
from config.shape_db_moreshape import ShapeDB

import matplotlib.pyplot as plt

setCartRos = rospy.ServiceProxy('/robot2_SetCartesian', robot_SetCartesian)
setZone = rospy.ServiceProxy('/robot2_SetZone', robot_SetZone)

def setCart(pos, ori):
    param = (np.array(pos) * 1000).tolist() + ori
    print 'setCart', param
    #pause()
    setCartRos(*param)

def addBuffer(pos, ori):
    param = (np.array(pos) * 1000).tolist() + ori
    addBufferRos(*param)

def pause():
    print 'Press any key to continue'
    raw_input()

def norm(vect):
    vect = np.array(vect)
    return np.sqrt(np.dot(vect, vect))
    
def poselist2mat(pose):
    return np.dot(tfm.translation_matrix(pose[0:3]), tfm.quaternion_matrix(pose[3:7]))

def mat2poselist(mat):
    pos = tfm.translation_from_matrix(mat)
    quat = tfm.quaternion_from_matrix(mat)
    return pos.tolist() + quat.tolist()

def wait_for_ft_calib():
    ROS_Wait_For_Msg('/netft_data', geometry_msgs.msg.WrenchStamped).getmsg()

import os
import errno

def make_sure_path_exists(path):
    try:
        os.makedirs(path)
    except OSError as exception:
        if exception.errno != errno.EEXIST:
            raise

def recover(obj_frame_id, global_frame_id, z, slot_pos_obj, reset):
    global globalvel
    global global_slow_vel
    zup = z + 0.03
    ori = [0, 0, 1, 0]
    center_world = [0.35, 0, 0]
    slot_pos_obj = slot_pos_obj + [0]
    if reset:
        # move above the slot
        pos_recover_probe_world = coordinateFrameTransform(slot_pos_obj, obj_frame_id, global_frame_id, listener)
        pos_recover_probe_world[2] = zup
        setCart(pos_recover_probe_world, ori)
        
        # speed down
        setSpeed(tcp=global_slow_vel, ori=1000)
        
        # move down to the slot    
        pos_recover_probe_world = coordinateFrameTransform(slot_pos_obj, obj_frame_id, global_frame_id, listener)
        pos_recover_probe_world[2] = z
        setCart(pos_recover_probe_world, ori)
        #pause()
        
        # move to the world center
        pos_recover_probe_target_world = center_world
        pos_recover_probe_target_world[2] = z
        setCart(pos_recover_probe_target_world, ori)
        
        # speed up
        setSpeed(tcp=globalvel, ori=1000)
    
    # move to the world center
    pos_recover_probe_target_world = center_world
    pos_recover_probe_target_world[2] = zup+0.03  # up more to let vicon see the marker
    setCart(pos_recover_probe_target_world, ori)
    
def polyapprox(shape, s):
    ss = shape[0]
    accu = []
    length = 0
    for i in range(len(ss)):
        accu.append(norm(np.array(ss[(i+1) % len(ss)])-np.array(ss[i])) + length)
        length = accu[-1]
    targetlength = s*length
    ind = 0
    print 'len(accu)', len(accu)
    
    for i in range(len(ss)):
        if accu[i] > targetlength:
            ind = i
            break
    
    print 'ind', ind
    
    seglength = norm(np.array(ss[(ind+1) % len(ss)])-np.array(ss[ind]))
    t = (targetlength-accu[ind]) / seglength
    pos = np.array(ss[ind]) * (1-t) +  np.array(ss[(ind+1) % len(ss)]) * t
    pos = np.append(pos, [0])
    tangent = np.array(ss[(ind+1) % len(ss)])-np.array(ss[ind])
    normal = np.array([tangent[1], -tangent[0]]) 
    normal = normal / norm(normal)  # normalize it
    normal = np.append(normal, [0])
    print 'len(pos)' , len(pos)
    return (pos, normal)

# check whether the probe will hit the object
def polyapprox_check_collision(shape, pos_start_probe_object, probe_radius):
    ss = shape[0]
    # find the closest point on the shape to pos_start_probe_object
    min_dist = 10000
    min_ind = 0
    safety_margin = 0.005
    for i in range(len(ss)):
        dist = norm(np.array(ss[i]) - np.array(pos_start_probe_object[0:2]))
        if dist < min_dist:
            min_dist = dist
            min_ind = i
            
    tangent = np.array(ss[(min_ind+1) % len(ss)])-np.array(ss[min_ind])
    normal = np.array([tangent[1], -tangent[0]]) # pointing out of shape
    normal = normal / norm(normal)  # normalize it
    d = np.dot(normal, np.array(pos_start_probe_object[0:2]) - np.array(ss[min_ind]) )
    if d < probe_radius + safety_margin:
        return True
    else:
        return False

import optparse
def main(argv):
    # prepare the proxy, listener
    
    parser = optparse.OptionParser()
    parser.add_option('-s', action="store", dest='shape_id', 
                      help='The shape id e.g. rect1-rect3', default='rect1')
    parser.add_option('-r', '--real', action="store_true", dest='real_exp', 
                      help='Do the real experiment space', 
                      default=False)
    parser.add_option('', '--slow', action="store_true", dest='slow', 
                      help='Set slower global speed', 
                      default=False)
                      
    (opt, args) = parser.parse_args()
    
    # set the parameters
    global globalvel
    global global_slow_vel
    globalvel = 300           # speed for moving around
    global_slow_vel = 30
    if opt.slow: globalvel = global_slow_vel
    ori = [0, 0, 1, 0]
    z = 0.218                 # the height above the table probe1: 0.29 probe2: 0.218
    surface_thick = 0.01158   # 0.01158 for plywood
    z = z + surface_thick
    z_recover = 0.2285 + surface_thick 
    zup = z + 0.08            # the prepare and end height
    probe_radius = 0.004745   # probe1: 0.00626/2 probe2: 0.004745
    dist_before_contact = 0.03 
    dist_after_contact = 0.05
    skip_when_exists = True
    reset_freq = 1


    global_frame_id = '/map'
    
    # parameters about the surface
    surface_id = 'plywood'
    
    # parameters about object
    shape_id = opt.shape_id
    shape_db = ShapeDB()
        
    shape_type = shape_db.shape_db[shape_id]['shape_type']
    shape = shape_db.shape_db[shape_id]['shape']
        
    obj_frame_id = shape_db.shape_db[shape_id]['frame_id']
    obj_slot = shape_db.shape_db[shape_id]['slot_pos']

    # space for the experiment
    real_exp = opt.real_exp
    if real_exp:
        acc_speed = np.linspace(20,1000,100)  # for acc
        speeds = reversed([20, 50, 100, 200, 400])
        #speeds = reversed([20, 50, 100, 200, 400, -1000])
        if shape_type == 'poly':
            side_params = np.linspace(0, 1, 11)  
        else:
            side_params = np.linspace(0,1,40,endpoint=False)
        
        angles = np.linspace(-pi/180.0*80.0, pi/180*80, 9)  
    
    
    # hack
    limit = 100
    cnt = 0
    
    # enumerate the speed
    for v in speeds:
        # enumerate the side we want to push
        for i in range(len(shape)):
            # enumerate the contact point that we want to push
            for s in side_params:
                if shape_type == 'poly':
                    pos = np.array(shape[i]) *s + np.array(shape[(i+1) % len(shape)]) *(1-s)
                    #pos = np.array(shape[i]) *(1-s) + np.array(shape[(i+1) % len(shape)]) *(s)   -> do it in next iteration
                    pos = np.append(pos, [0])
                    tangent = np.array(shape[(i+1) % len(shape)]) - np.array(shape[i])
                    normal = np.array([tangent[1], -tangent[0]]) 
                    normal = normal / norm(normal)  # normalize it
                    normal = np.append(normal, [0])
                elif shape_type == 'ellip':
                    (a,b) = shape[0][0], shape[0][1]
                    pos = [shape[0][0] * np.cos(s*2*np.pi), shape[0][1] * np.sin(s*2*np.pi), 0]
                    normal = [np.cos(s*2*np.pi)/a, np.sin(s*2*np.pi)/b, 0]
                    normal = normal / norm(normal)  # normalize it
                elif shape_type == 'polyapprox':
                    pos, normal = polyapprox(shape, s)
                    
                # enumerate the direction in which we want to push
                for t in angles:
                    bagfilename = 'motion_surface=%s_shape=%s_v=%.0f_i=%.3f_s=%.3f_t=%.3f.bag' % (surface_id, shape_id, v, i, s, t)
                    bagfilepath = bagfilename
                    # find the probe pos in contact in object frame
                    pos_probe_contact_object = pos + normal * probe_radius
                    # find the start point
                    direc = np.dot(tfm.euler_matrix(0,0,t) , normal.tolist() + [1])[0:3] # in the direction of moving out
                    pos_start_probe_object = pos_probe_contact_object + direc * dist_before_contact
                    
                    pos_end_probe_object = pos_probe_contact_object - direc * dist_after_contact
                    
                    
                    if shape_type == 'polyapprox' and polyapprox_check_collision(shape, pos_start_probe_object, probe_radius):
                        print bagfilename, 'will be in collision', 'skip'
                        continue
                    
                    plt.scatter(pos_start_probe_object[0], pos_start_probe_object[1])
                    print pos_start_probe_object
    
    plt.scatter(np.array(shape[0])[:,0], np.array(shape[0])[:,1], c=[0.1]*len(shape[0]))
    #plt.scatter(pos_start_probe_object[0], pos_start_probe_object[1])
    plt.show()

def terminate_ros_node(s):
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for str in list_output.split("\n"):
        if (str.startswith(s)):
            os.system("rosnode kill " + str)


if __name__=='__main__':
    main(sys.argv)

