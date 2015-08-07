#!/usr/bin/env python

# Peter KT Yu, Aug 2015
import sys
import numpy as np
from ik.roshelper import ROS_Wait_For_Msg
from ik.roshelper import lookupTransform
from geometry_msgs.msg import WrenchStamped
import tf
import tf.transformations as tfm
import rospy
import json
import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *
import sensor_msgs.msg
import geometry_msgs.msg

setCartRos = rospy.ServiceProxy('/robot2_SetCartesian', robot_SetCartesian)
def setCart(pos, ori):
    param = (np.array(pos) * 1000).tolist() + ori
    print 'setCart', param
    #pause()
    setCartRos(*param)

def pause():
    print 'Press any key to continue'
    raw_input()


def main(argv):
    # prepare the proxies, listener
    rospy.init_node('contour_follow', anonymous=True)
    listener = tf.TransformListener()
    
    # set the parameters
    z = 0.29
    limit = 10000
    ori = [0, 0.7071, 0.7071, 0]
    threshold = 0.1  # need to be tuned
    probe_radis = 0.00626/2
    
    # 0. move to startPos
    start_pos = [0.3, 0.3, z + 0.05]
    setCart(start_pos,ori)
    
    start_pos = [0.3, 0.3, z]
    setCart(start_pos,ori)
    curr_pos = start_pos
    
    # 1. move in -y direction till contact -> normal
    direc = np.array([0,-0.001,0])
    normal = [0,0,0]
    while True:
        curr_pos = np.array(curr_pos) + direc
        setCart(curr_pos,ori)
        ft = ROS_Wait_For_Msg('/netft_data', geometry_msgs.msg.WrenchStamped).getmsg()
        
        (pos_trasform,ori_trasform) = lookupTransform('base_link', 'link_ft', listener)
        
        rotmat = tfm.quaternion_matrix(ori_trasform)
        f = ft.wrench.force
        ft_global = np.dot(rotmat, [f.x, f.y, f.z, 1.0])
        #pause()
        # in contact
        # if np.sqrt(ft_global[0]**2 + ft_global[1]**2) > threshold:
            # normal = ft_global[0:3] / np.sqrt(np.dot(ft_global, ft_global))
            # break
    pause()
    
    # 2. use the normal to move along the block
    index = 0
    contact_pt = []
    contact_nm = []
    while True:
        # 2.1 move 
        direc = np.dot(tfm.euler_matrix(0,0,1.57) , normal)[0:3]
        curr_pos = np.array(curr_pos) + direc
        setCart(curr_pos,ori)
        # 
        (pos_trasform,ori_trasform) = listener.lookupTransform('base_link','link_ft',t)
        rotmat = tfm.quaternion_matrix(ori_trasform)
        f = ft.wrench.force
        ft_global = np.dot(rotmat, [f.x, f.y, f.z, 1.0])
        if np.sqrt(ft_global[0]**2 + ft_global[1]**2) > threshold:
            # ft_global[2] = 0  ??
            normal = ft_global[0:3] / np.sqrt(np.dot(ft_global, ft_global))
            contact_nm.append(normal)
            contact_pt.append(curr_pos + normal * probe_radis)
        
        if len(contact_pt) >= limit:
            break
    
    # save contact_nm and contact_pt as json file
    with open(os.environ['PNPUSHDATA_BASE']+'/data.json', 'w') as outfile:
        json.dump({'contact_pt': contact_pt, 'contact_nm': contact_nm}, outfile)
        
    sio.savemat(os.environ['PNPUSHDATA_BASE']+'/data.mat', {'contact_pt':contact_pt, 'contact_nm': contact_nm})
    
if __name__=='__main__':
    main(sys.argv)


#rosservice call /robot2_SetSpeed 10 1
#rosservice call /robot2_SetZone "mode: 1"
