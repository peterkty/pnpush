#!/usr/bin/env python

# Peter KT Yu, Aug 2015
# Plot the trajectory

import numpy as np
import json

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection

from config.shape_db import ShapeDB

import tf.transformations as tfm
from ik.helper import matrix_from_xyzquat

def plot(data):

    fig, ax = plt.subplots()
    probe_radius = 0.004745   # probe1: 0.00626/2 probe2: 0.004745
    sub = 20                 # subsample rate
    #data['tip_poses']
    #data['ft_wrench']
    #data['object_pose']
    tip_pose = data['tip_poses']
    
    patches = []
    # add the probes as circle
    for i in range(0, len(tip_pose), sub):
        circle = mpatches.Circle(tip_pose[i][1:3], probe_radius, ec="none")
        patches.append(circle)

    collection = PatchCollection(patches, cmap=plt.cm.hsv, alpha=0.3)
    ax.add_collection(collection)
    
    
    # add the object as polygon
    sdb = ShapeDB()
    shape_polygon = shape_db.shape_db[shape_id]['shape_poly'] # shape of the objects presented as polygon.
    shape_polygon_3d = np.hcat(np.array(shape_polygon).T, np.zeros(len(shape_polygon), 1), np.ones(len(shape_polygon), 1))
    
    object_pose = data['object_pose']
    # add the probes as circle
    for i in range(0, len(object_pose), sub):
        quaternion = object_pose[4:8]
        
        T = matrix_from_xyzquat(object_pose[1:4], object_pose[4:8])
        shape_polygon_3d_world = np.dot(T, shape_polygon_3d)
        
        Polygon(shape_polygon_3d_world.T[:][0:2], closed=True)
    
    plt.subplots_adjust(left=0, right=1, bottom=0, top=1)
    plt.axis('equal')
    #plt.axis('off')

    plt.show()

def main(argv):
    bag_filepath = argv[1]
    with open(bag_filepath) as data_file:    
        data = json.load(data_file)
    
    shape_id = 'rect1'
    plot(data, shape_id)
    

if __name__=='__main__':
    import sys
    main(sys.argv)
    
