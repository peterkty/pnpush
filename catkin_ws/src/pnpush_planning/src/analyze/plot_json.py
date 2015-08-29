#!/usr/bin/env python

# Peter KT Yu, Aug 2015
# Plot the trajectory

import numpy as np
import json

import matplotlib
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection

from config.shape_db import ShapeDB

import tf.transformations as tfm
from ik.helper import matrix_from_xyzquat
from matplotlib.pyplot import savefig

def plot(data, shape_id, figfname):



    fig, ax = plt.subplots()
    probe_radius = 0.004745   # probe1: 0.00626/2 probe2: 0.004745
    

    v = int(figfname.split('_')[-4].split('=')[1])
    sub = int(30 / (v / 20.0))                 # subsample rate
    #data['tip_poses']
    #data['ft_wrench']
    #data['object_pose']
    tip_pose = data['tip_poses']
    
    patches = []
    
    
    # add the object as polygon
    shape_db = ShapeDB()
    shape_polygon = shape_db.shape_db[shape_id]['shape_poly'] # shape of the objects presented as polygon.
    shape_polygon_3d = np.hstack((np.array(shape_polygon), np.zeros((len(shape_polygon), 1)), np.ones((len(shape_polygon), 1))))
    
    object_pose = data['object_pose']
    
    invT0 = np.linalg.inv(matrix_from_xyzquat(object_pose[0][1:4], object_pose[0][4:8]))


    print 'object_pose', len(object_pose), 'tip_pose', len(tip_pose)

        
    for i in (range(0, len(object_pose), sub)):
        
        T = matrix_from_xyzquat(object_pose[i][1:4], object_pose[i][4:8])
        shape_polygon_3d_world = np.dot(np.dot(invT0, T), shape_polygon_3d.T)
        
        obj = mpatches.Polygon(shape_polygon_3d_world.T[:,0:2], closed=True, color='blue', alpha=0.05)
        ax.add_patch(obj)
        #print 'append', obj
    
    # add the probes as circle
    for i in (range(0, len(tip_pose), sub)):
        tip_pose_0 = np.dot(invT0, tip_pose[i][1:4]+[1])
        circle = mpatches.Circle(tip_pose_0[0:2], probe_radius, ec="none", color='red', alpha=0.5)
        #patches.append(circle)
        ax.add_patch(circle)

    # render it
    #collection = PatchCollection(patches, alpha=0.1)
    #ax.add_collection(collection)
    #plt.subplots_adjust(left=0, right=1, bottom=0, top=1)
    plt.axis([-0.1, 0.1, -0.1, 0.1])
    
    plt.axis('equal')
    #plt.axis('off')
    
    if figfname is not None:
        plt.savefig(figfname)

def main(argv):
    if len(argv) < 2:
        print 'Usage: analyze_json.py *.json'
        return
        
    bag_filepath = argv[1]
    with open(bag_filepath) as data_file:    
        data = json.load(data_file)
    
    shape_id = 'rect1'
    plot(data, shape_id, bag_filepath.replace('.json', '.png'))


if __name__=='__main__':
    import sys
    main(sys.argv)
    
