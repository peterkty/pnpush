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

from config.shape_db import ShapeDB

def vizBlock(pose, mesh, frame_id):
    # prepare block visualization
    global vizpub
    meshmarker = createMeshMarker(mesh, 
                 offset=tuple(pose[0:3]), rgba=(0.5,0.5,0.5,1),
                 orientation=tuple(pose[3:7]), frame_id=frame_id, marker_id = 2)
    vizpub.publish(meshmarker)

import optparse
def main(argv):
    parser = optparse.OptionParser()
    parser.add_option('-s', action="store", dest='shape_id', 
                      help='The shape id e.g. rect1-rect3', default='rect1')
    
                      
    (opt, args) = parser.parse_args()
    vicon_ball_size = 0.01415  # in diameter

    rospy.init_node('vicon_object_visulizer')
    listener = tf.TransformListener()
    global vizpub
    vizpub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    
    
    # parameters about object
    shape_id = opt.shape_id  # should be a rosparam
    shape_db = ShapeDB()
    mesh = shape_db.shape_db[shape_id]['mesh']
    frame_id = shape_db.shape_db[shape_id]['frame_id']
    obj_slot = shape_db.shape_db[shape_id]['slot_pos']
    thickness = shape_db.shape_db[shape_id]['thickness']
    vicon_marker_plate_thick = 0.002
    
    obj_des_wrt_vicon = [0,0,-(thickness/2 + vicon_ball_size/2 + vicon_marker_plate_thick + 0.0036) ,0,0,0,1]  # from vicon to the block (a slight difference in z)
        
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        # get box pose from vicon
        try:
            vizBlock(obj_des_wrt_vicon, mesh, frame_id)
        except:
            print 'object not in view'
        
        r.sleep()

if __name__=='__main__':
    main(sys.argv)
