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
    #data['tip_poses']
    #data['ft_wrench']
    #data['object_pose']
    
    probe_radius = 0.004745   # probe1: 0.00626/2 probe2: 0.004745
    
    fig, ax = plt.subplots()
    fig.set_size_inches(7,7)

    v = int(figfname.split('_')[-4].split('=')[1])
    sub = int(30 / (v / 20.0))                 # subsample rate
    tip_pose = data['tip_poses']
    
    patches = []
    
    # add the object as polygon
    shape_db = ShapeDB()
    shape = shape_db.shape_db[shape_id]['shape'] # shape of the objects presented as polygon.
    shape_type = shape_db.shape_db[shape_id]['shape_type']
    if shape_type == 'poly':
        shape_polygon_3d = np.hstack((np.array(shape), np.zeros((len(shape), 1)), np.ones((len(shape), 1))))
    elif shape_type == 'ellip':
        shape = shape[0]
    elif shape_type == 'polyapprox':
        shape_polygon_3d = np.hstack((np.array(shape[0]), np.zeros((len(shape[0]), 1)), np.ones((len(shape[0]), 1))))
    
    object_pose = data['object_pose']
    
    invT0 = np.linalg.inv(matrix_from_xyzquat(object_pose[0][1:4], object_pose[0][4:8]))


    print 'object_pose', len(object_pose), 'tip_pose', len(tip_pose)

        
    r = (range(0, len(object_pose), sub)) + [len(object_pose)-1]
    for i in r:
        
        T = matrix_from_xyzquat(object_pose[i][1:4], object_pose[i][4:8])
        
        if i == 0:
            alpha , fill = (0.3, True)
        elif i == r[-1]:
            alpha , fill = (0.6, True)
        else:
            alpha , fill = (0.6, False)
        
        ec, fc = 'black','orangered'
        if shape_type == 'poly' or shape_type == 'polyapprox':
            shape_polygon_3d_world = np.dot(np.dot(invT0, T), shape_polygon_3d.T)
            obj = mpatches.Polygon(shape_polygon_3d_world.T[:,0:2], closed=True, fc=fc, ec=ec, alpha=alpha, fill=fill, linewidth=1, linestyle='solid')
        elif shape_type == 'ellip':
            T_T0 = np.dot(invT0, T)
            scale, shear, angles, trans, persp = tfm.decompose_matrix(T_T0)
            obj = mpatches.Ellipse(trans[0:2], shape[0]*2, shape[1]*2, angle=angles[2], fc=fc, ec=ec, alpha=alpha, fill=fill, linewidth=1, linestyle='solid')
            
        ax.add_patch(obj)
    
    # add the probes as circle
    r = (range(0, len(tip_pose), sub)) + [len(tip_pose)-1]
    for i in r:
        tip_pose_0 = np.dot(invT0, tip_pose[i][1:4]+[1])
        if i == 0:
            alpha , fill = (0.8, False)
        elif i == r[-1]:
            alpha , fill = (0.8, False)
        else:
            alpha , fill = (0.8, False)
        circle = mpatches.Circle(tip_pose_0[0:2], probe_radius, color='black', alpha=alpha, fill=fill, linewidth=1, linestyle='solid')
            
        ax.add_patch(circle)

    # render it
    plt.axis([-0.15, 0.15, -0.15, 0.15])
    plt.axis('off')
    
    if figfname is not None:
        plt.savefig(figfname)

def getfield_from_filename(figname, field):
    return figname[figname.find(field):].split('_')[0].split('=')[1]

def main(argv):
    if len(argv) < 2:
        print 'Usage: plot_raw_json.py *.json'
        return
        
    bag_filepath = argv[1]
    with open(bag_filepath) as data_file:    
        data = json.load(data_file)
    
    #shape_id = 'rect1'
    figname = bag_filepath.replace('.json', '.png')
    shape_id = getfield_from_filename(figname, 'shape')
    plot(data, shape_id, figname)


if __name__=='__main__':
    import sys
    main(sys.argv)
    
