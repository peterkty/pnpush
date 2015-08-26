#!/usr/bin/env python

# this is to visualize the block in rviz

import tf.transformations as tfm
import numpy as np
from ik.roshelper import lookupTransform
import tf
import rospy
from tf.broadcaster import TransformBroadcaster

from visualization_msgs.msg import Marker
from marker_helper import createMeshMarker
import sys

def vizBlock(pose):
    # prepare block visualization
    global vizpub
    meshmarker = createMeshMarker('package://pnpush_config/models/object_meshes/SteelBlock.stl', 
                 offset=tuple(pose[0:3]), rgba=(0.5,0.5,0.5,1),
                 orientation=tuple(pose[3:7]), frame_id='vicon/SteelBlock/SteelBlock')
    vizpub.publish(meshmarker)
    

def main(argv):
    obj_des_wrt_vicon = [0,0,-(9.40/2/1000+14.15/2/1000),0,0,0,1]  # from vicon to the block (a slight difference in z)

    rospy.init_node('vicon_object_visulizer')
    listener = tf.TransformListener()
    global vizpub
    vizpub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    #br = TransformBroadcaster()
        
    rospy.sleep(1)
    while True:
        # get box pose from vicon
        try:
            (box_pos, box_quat) = lookupTransform('map', 'vicon/SteelBlock/SteelBlock', listener)
            vizBlock(obj_des_wrt_vicon)
        except:
            print 'object not in view'
        
        rospy.sleep(0.01)

if __name__=='__main__':
    main(sys.argv)
