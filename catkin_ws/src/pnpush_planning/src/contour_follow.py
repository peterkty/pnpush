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
import os
import scipy.io as sio
from visualization_msgs.msg import Marker
from marker_helper import createMeshMarker
from marker_helper import createPointMarker
from marker_helper import createArrowMarker
from marker_helper import createSphereMarker

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
    marker = createSphereMarker(point=pos, color=[0, 0, 1, 0.5])
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

def mat2poselist(pose):
    pos = tfm.translation_from_matrix(mat)
    quat = tfm.quaternion_from_matrix(mat)
    return pos.tolist() + quat.tolist()
        
def main(argv):
    # prepare the proxy, listener
    global listener
    global vizpub
    rospy.init_node('contour_follow', anonymous=True)
    listener = tf.TransformListener()
    vizpub = rospy.Publisher("visualization_marker", Marker)
    setSpeed(tcp=100, ori=30)
    # set the parameters
    z = 0.29   # the height above the table
    limit = 1000  # number of data points to be collected
    ori = [0, 0.7071, 0.7071, 0]
    threshold = 0.5  # the threshold force for contact, need to be tuned
    probe_radis = 0.00626/2
    step_size = 0.0002
    obj_des_wrt_vicon = [-0.01406426, -0.00907471, -0.02657346, -8.42456325e-05,  -1.77929224e-04,  -2.94718714e-03,  9.99995638e-01]
    
    # visualize the block 
    vizBlock(obj_des_wrt_vicon)
    
    # 0. move to startPos
    start_pos = [0.3, 0.06, z + 0.05]
    setCart(start_pos,ori)
    
    start_pos = [0.3, 0.06, z]
    setCart(start_pos,ori)
    curr_pos = start_pos
    # 0.1 zero the ft reading
    rospy.sleep(3)  
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
        vizBlock(obj_des_wrt_vicon)
        
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
        print index, '[ft explore]', ft
        # get box pose from vicon
        (box_pos, box_quat) = lookupTransform('base_link', 'vicon/SteelBlock/SteelBlock', listener)
        # correct box_pose
        box_mat =  np.dot(poselist2mat(obj_des_wrt_vicon), poselist2mat(box_pos.tolist() + box_quat.tolist()))
        box_pose_des_global = box_mat
        print 'box_pose', box_pose_des_global
        
        vizBlock(obj_des_wrt_vicon)
        #print 'box_pos', box_pos, 'box_quat', box_quat
                
        if norm(ft[0:2]) > threshold:
            # transform ft data to global frame
            ft_global = transformFt2Global(ft)
            ft_global[2] = 0  # we don't want noise from z
            normal = ft_global[0:3] / norm(ft_global)
            contact_nms.append(normal.tolist())
            contact_pt = curr_pos - normal * probe_radis
            contact_pts.append(contact_pt.tolist())
            contact_pt[2] = 0.01
            vizPoint(contact_pt.tolist())
            vizArrow(contact_pt.tolist(), (contact_pt + normal * 0.1).tolist())
            all_contact.append(contact_pt.tolist()[0:2] + [0] + normal.tolist()[0:2] + [0] + list(box_pos) + list(box_quat) + curr_pos.tolist())
            index += 1
        
        if len(contact_pts) > limit:
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
    sio.savemat(os.environ['PNPUSHDATA_BASE']+'/all_contact_real.mat', {'all_contact': all_contact})
    
    setSpeed(tcp=100, ori=30)
    # 3. move back to startPos
    start_pos = [0.3, 0.06, z + 0.05]
    setCart(start_pos,ori)
    
if __name__=='__main__':
    main(sys.argv)


#rosservice call /robot2_SetSpeed 10 1
#rosservice call /robot2_SetZone "mode: 1"


# import tf.transformations as tfm
# import numpy as np
# xr = [0.41402, 0.009153, 0.02657, 8.4246e-05, 0.00017793, 0.0029472, 1]
# x = [0.4, 0, 0, 0, 0, 0 ,1]
# y = np.dot ( np.linalg.inv( np.dot(tfm.translation_matrix(x[0:3]), tfm.quaternion_matrix(x[3:7])) ) , np.dot(tfm.translation_matrix(xr[0:3]), tfm.quaternion_matrix(xr[3:7])) )
# y = np.linalg.inv(y)
# q = tfm.quaternion_from_matrix(y)
# t = tfm.translation_from_matrix(y)
