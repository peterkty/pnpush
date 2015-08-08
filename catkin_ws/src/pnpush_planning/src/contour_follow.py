#!/usr/bin/env python

# Peter KT Yu, Aug 2015
# Do contour following with the poker around the shape using the force
# torque reading.

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

setCartRos = rospy.ServiceProxy('/robot2_SetCartesian', robot_SetCartesian)
setZero = rospy.ServiceProxy('/zero', Zero)

def setCart(pos, ori):
    param = (np.array(pos) * 1000).tolist() + ori
    print 'setCart', param
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
     
def main(argv):
    # prepare the proxy, listener
    global listener
    rospy.init_node('contour_follow', anonymous=True)
    listener = tf.TransformListener()
    
    
    setSpeed(tcp=100, ori=30)
    # set the parameters
    z = 0.29   # the height above the table
    limit = 10000  # number of data points to be collected
    ori = [0, 0.7071, 0.7071, 0]
    threshold = 0.5  # the threshold force for contact, need to be tuned
    probe_radis = 0.00626/2
    step_size = 0.0002
    
    # 0. move to startPos
    start_pos = [0.3, 0.15, z + 0.05]
    setCart(start_pos,ori)
    
    start_pos = [0.3, 0.15, z]
    setCart(start_pos,ori)
    curr_pos = start_pos
    # 0.1 zero the ft reading
    rospy.sleep(10)  
    setZero()
    
    # 1. move in -y direction till contact -> normal
    setSpeed(tcp=30, ori=30)
    direc = np.array([0, -step_size, 0])
    normal = [0,0,0]
    while True:
        curr_pos = np.array(curr_pos) + direc
        setCart(curr_pos, ori)
        # let the ft reads the force in static, not while pushing
        rospy.sleep(0.1)  
        ft = ftmsg2list(ROS_Wait_For_Msg('/netft_data', geometry_msgs.msg.WrenchStamped).getmsg())
        print '[ft explore]', ft
        
        # If in contact, break
        if norm(ft[0:2]) > threshold:
            # transform ft data to global frame
            ft_global = transformFt2Global(ft)
            ft_global[2] = 0  # we don't want noise from z
            normal = ft_global[0:3] / norm(ft_global)
            break
    #pause()
    
    # 2. use the normal to move along the block
    setSpeed(tcp=20, ori=30)
    index = 0
    contact_pts = []
    contact_nms = []
    all_contact = []
    while True:
        # 2.1 move 
        direc = np.dot(tfm.euler_matrix(0,0,2) , normal.tolist() + [1])[0:3]
        curr_pos = np.array(curr_pos) + direc * step_size
        setCart(curr_pos, ori)
        
        # let the ft reads the force in static, not while pushing
        rospy.sleep(0.1)
        ft = ftmsg2list(ROS_Wait_For_Msg('/netft_data', geometry_msgs.msg.WrenchStamped).getmsg())
        print '[ft explore]', ft
        # get box pose from vicon
        box_pos = [0,0,0]
        box_quat = [0,0,0,1]
        
        if norm(ft[0:2]) > threshold:
            # transform ft data to global frame
            ft_global = transformFt2Global(ft)
            ft_global[2] = 0  # we don't want noise from z
            normal = ft_global[0:3] / norm(ft_global)
            contact_nms.append(normal)
            contact_pt = curr_pos + normal * probe_radis
            contact_pts.append(contact_pt)
            all_contact.append(contact_pt.tolist() + [0] + normal.tolist() + [0] + box_pos + box_quat + curr_pos.tolist())
        
        if len(contact_pt) >= limit:
            break
    
      #all_contact(1:2,:);  % x,y of contact position
      #all_contact(4:5,:);  % x,y contact normal
      #all_contact(7:9,:);  % box x,y
      #all_contact(10:13,:);  % box quaternion
      #all_contact(14:16,:);  % pusher position
    
    
    # save contact_nm and contact_pt as json file
    with open(os.environ['PNPUSHDATA_BASE']+'/all_contact_real.json', 'w') as outfile:
        json.dump({'contact_pts': contact_pts, 'contact_nms': contact_nms}, outfile)
        
    # save all_contact as mat file
    sio.savemat(os.environ['PNPUSHDATA_BASE']+'/all_contact_real.mat', all_contact)
    
if __name__=='__main__':
    main(sys.argv)


#rosservice call /robot2_SetSpeed 10 1
#rosservice call /robot2_SetZone "mode: 1"
