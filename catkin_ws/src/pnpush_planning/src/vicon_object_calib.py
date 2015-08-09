#!/usr/bin/env python

# this is to find out the transform between desired frame and vicon frame on the object

import tf.transformations as tfm
import numpy as np
from ik.roshelper import lookupTransform
import tf
import rospy
from tf.broadcaster import TransformBroadcaster

from visualization_msgs.msg import Marker
from marker_helper import createMeshMarker

def poselist2mat(pose):
    return np.dot(tfm.translation_matrix(pose[0:3]), tfm.quaternion_matrix(pose[3:7]))

def mat2poselist(mat):
    pos = tfm.translation_from_matrix(mat)
    quat = tfm.quaternion_from_matrix(mat)
    return pos.tolist() + quat.tolist()

def vizBlock(pose):
    # prepare block visualization
    global vizpub
    meshmarker = createMeshMarker('package://pnpush_config/models/object_meshes/SteelBlock.stl', 
                              offset=tuple(pose[0:3]), rgba=(0.5,0.5,0.5,1),
                              orientation=tuple(pose[3:7]), frame_id='vicon/SteelBlock/SteelBlock')
    vizpub.publish(meshmarker)
    rospy.sleep(0.05)
    
xr = [0.0077503, 0.0044662, 0.016436, -0.00095437, 0.0012349, -0.0053827, 0.99998]
xk = [0, 0, 0, 0, 0, 0 ,1]
#y = np.dot ( np.linalg.inv( np.dot(tfm.translation_matrix(x[0:3]), tfm.quaternion_matrix(x[3:7])) ) , np.dot(tfm.translation_matrix(xr[0:3]), tfm.quaternion_matrix(xr[3:7])) )
xmat = np.dot( np.linalg.inv(poselist2mat(xr)), poselist2mat(xk) )
xmat2 = np.linalg.solve( poselist2mat(xr), poselist2mat(xk))
x = mat2poselist(xmat)
obj_des_wrt_vicon = x

print x
print mat2poselist(np.dot(xmat, poselist2mat(xr)))
print mat2poselist(np.dot(xmat2, poselist2mat(xr)))

rospy.init_node('contour_follow', anonymous=True)
listener = tf.TransformListener()
vizpub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
br = TransformBroadcaster()
    
rospy.sleep(1)
while True:
    # get box pose from vicon
    (box_pos, box_quat) = lookupTransform('map', 'vicon/SteelBlock/SteelBlock', listener)
    # correct box_pose
    box_mat =  np.dot(poselist2mat(list(box_pos) + list(box_quat)), poselist2mat(obj_des_wrt_vicon))
    #box_mat =  np.dot(poselist2mat(obj_des_wrt_vicon), poselist2mat(list(box_pos) + list(box_quat)))
    box_pose_des_global = mat2poselist(box_mat)
    #print 'box_pose', box_pose_des_global
    
    vizBlock(obj_des_wrt_vicon)
    br.sendTransform(box_pose_des_global[0:3], box_pose_des_global[3:7], rospy.Time.now(), "SteelBlockDesired", "map")
    rospy.sleep(0.1)
