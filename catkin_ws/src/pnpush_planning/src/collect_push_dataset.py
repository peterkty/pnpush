#!/usr/bin/env python

# Peter KT Yu, Aug 2015
# automate the process of pushing against an object and record the data

import sys
import numpy as np
from ik.roshelper import ROS_Wait_For_Msg
from ik.roshelper import lookupTransform
from ik.ik import setSpeed
from geometry_msgs.msg import WrenchStamped
import tf
import tf.transformations as tfm
import rospy
import json
import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *
roslib.load_manifest("netft_rdt_driver")
from netft_rdt_driver.srv import Zero
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

setCartRos = rospy.ServiceProxy('/robot2_SetCartesian', robot_SetCartesian)
setZero = rospy.ServiceProxy('/zero', Zero)
setZone = rospy.ServiceProxy('/robot2_SetZone', robot_SetZone)

def setCart(pos, ori):
    param = (np.array(pos) * 1000).tolist() + ori
    #print 'setCart', param
    #pause()
    setCartRos(*param)

def pause():
    print 'Press any key to continue'
    raw_input()

def transformFt2Global(ftlist):
    global listener
    # transform ft data to global frame
    (pos_trasform, ori_trasform) = lookupTransform('base_link', 'link_ft', listener)
    rotmat = tfm.quaternion_matrix(ori_trasform)
    ft_global = np.dot(rotmat, ftlist + [1.0])
    return ft_global[0:3].tolist()

def ftmsg2list(ftmsg):
    return [ftmsg.wrench.force.x,ftmsg.wrench.force.y,ftmsg.wrench.force.z]

def norm(vect):
    vect = np.array(vect)
    return np.sqrt(np.dot(vect, vect))

def vizBlock(pose):
    # prepare block visualization
    global vizpub
    meshmarker = createMeshMarker('package://pnpush_config/models/object_meshes/SteelBlock.stl', 
                              offset=tuple(pose[0:3]), rgba=(0.5,0.5,0.5,1),
                              orientation=tuple(pose[3:7]), frame_id='vicon/SteelBlock/SteelBlock')
    vizpub.publish(meshmarker)
    rospy.sleep(0.05)
    
def vizPoint(pos):
    # prepare block visualization
    global vizpub
    marker = createSphereMarker(offset=pos, color=[0, 0, 1, 0.5], scale=[0.01,0.01,0.01])
    vizpub.publish(marker)
    rospy.sleep(0.1)

def vizArrow(start, end):
    # prepare block visualization
    global vizpub
    marker = createArrowMarker(points=start+end, color=[1,0,0,1])
    vizpub.publish(marker)
    rospy.sleep(0.1)

def poselist2mat(pose):
    return np.dot(tfm.translation_matrix(pose[0:3]), tfm.quaternion_matrix(pose[3:7]))

def mat2poselist(mat):
    pos = tfm.translation_from_matrix(mat)
    quat = tfm.quaternion_from_matrix(mat)
    return pos.tolist() + quat.tolist()

def getAveragedFT():
    tmpft = np.array([0,0,0])
    nsample = 10
    for i in xrange(0,nsample):
        tmpft =  tmpft + np.array(ftmsg2list(ROS_Wait_For_Msg('/netft_data', geometry_msgs.msg.WrenchStamped).getmsg()))
    #print tmpft / nsample
    return (tmpft / nsample).tolist()

def main(argv):
    # prepare the proxy, listener
    global listener
    global vizpub
    rospy.init_node('contour_follow', anonymous=True)
    listener = tf.TransformListener()
    vizpub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    br = TransformBroadcaster()
    
    setSpeed(tcp=100, ori=30)
    setZone(0)
    # set the parameters
    ori = [0, 0.7071, 0.7071, 0]
    z = 0.218   # the height above the table probe1: 0.29 probe2: 0.218
    probe_radis = 0.004745   # probe1: 0.00626/2 probe2: 0.004745
    step_size = 0.0002
    obj_des_wrt_vicon = [0,0,-(9.40/2/1000+14.15/2/1000),0,0,0,1]
    dist_before_contact = 0.1 
    dist_after_contact = 0.02
    
    shape_polygon = [[-0.0985/2, -0.0985/2], [0.0985/2, -0.0985/2]] # shape of the objects presented as polygon.
    
    # visualize the block 
    for i in xrange(7):
        vizBlock(obj_des_wrt_vicon)
        rospy.sleep(0.1)
    
    pi = 3.14
    for i in len(shape_polygon) - 1:  # the side we want to push
        for s in np.linspace(0,1,5):  # the point that we want to push
            pos = np.array(shape_polygon[i]) *s + np.array(shape_polygon[i+1]) *(1-s)
            tangent = shape_polygon[i+1] - shape_polygon[i-1]
            normal = [tangent[1], -tangent[0]]  # need to make sure
            for t in np.linspace(pi/2, -pi/2, 3):
                # find the start point
                # find the end point
                
    
    # 0. move to startPos
    start_pos = [0.3, 0.06, z + 0.05]
    setCart(start_pos,ori)
    
    start_pos = [0.3, 0.06, z]
    setCart(start_pos,ori)
    curr_pos = start_pos
    # 0.1 zero the ft reading
    rospy.sleep(1)  
    setZero()
    rospy.sleep(3)
    

    
    
    # save contact_nm and contact_pt as json file
    with open(os.environ['PNPUSHDATA_BASE']+'/all_contact_real.json', 'w') as outfile:
        json.dump({'contact_pts': contact_pts, 'contact_nms': contact_nms}, outfile)

    # save all_contact as mat file
    sio.savemat(os.environ['PNPUSHDATA_BASE']+'/all_contact_real.mat', {'all_contact': all_contact})
    
    setSpeed(tcp=100, ori=30)
    # 3. move back to startPos
    start_pos = [0.3, 0.06, z + 0.05]
    setCart(start_pos,ori)
    
if __name__=='__main__':
    main(sys.argv)


#rosservice call /robot2_SetSpeed 10 1
#rosservice call /robot2_SetZone "mode: 1"



