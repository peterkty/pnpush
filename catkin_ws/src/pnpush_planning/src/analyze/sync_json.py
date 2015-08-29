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

import pdb
def extract2d_and_cleanup(data):

    probe_radius = 0.004745   # probe1: 0.00626/2 probe2: 0.004745
    
    tip_pose = data['tip_poses']
    object_pose = data['object_pose']
    ft = data['ft_wrench']
    
    # transformation to the first 
    invT0 = np.linalg.inv(matrix_from_xyzquat(object_pose[0][1:4], object_pose[0][4:8]))
    print 'object_pose', len(object_pose), 'tip_pose', len(tip_pose)

    sub = 1
    
    # object
    object_pose_2d = []
    for i in (range(0, len(object_pose), sub)):
        T = matrix_from_xyzquat(object_pose[i][1:4], object_pose[i][4:8])
        tip_pose_object0 = np.dot(invT0, T)
        scale, shear, angles, trans, persp = tfm.decompose_matrix(tip_pose_object0)
        #print 'trans', trans[0:2], 'angle', angles[2]
        time = object_pose[i][0]
        # don't add redundant data entry with the same time
        if(not(len(object_pose_2d) > 0 and time == object_pose_2d[-1][0] )):
            object_pose_2d.append([time] + trans[0:2] + [angles[2]])
    
    # probe
    tip_pose_2d = []
    for i in (range(0, len(tip_pose), sub)):
        tip_pose_0 = np.dot(invT0, tip_pose[i][1:4]+[1])
        #print 'trans', tip_pose_0[0:2]
        time = tip_pose[i][0]
        
        # don't add redundant data entry with the same time
        if(not(len(tip_pose_2d) > 0 and time == tip_pose_2d[-1][0] )):
            tip_pose_2d.append([time] + tip_pose_0[0:2].tolist())

    # ft, no redundency
    ft_2d = np.array(ft)[:,0:3].tolist()   # only need the force
    print 'object_pose_2d', len(object_pose_2d), 'tip_pose_2d', len(tip_pose_2d), 'ft_2d', len(ft_2d)
    
    data2d = {}
    data2d['tip_poses_2d'] = tip_pose_2d
    data2d['object_poses_2d'] = object_pose_2d
    data2d['force_2d'] = ft_2d
    return data2d


import pandas as pd

def resample_using_pandas(data):
    force_2d = data['force_2d']
    object_poses_2d = data['object_poses_2d']
    tip_poses_2d = data['tip_poses_2d']
    starttime = max(force_2d[0][0], object_poses_2d[0][0], force_2d[0][0])
    endtime = min(force_2d[-1][0], object_poses_2d[-1][0], force_2d[-1][0])
    
    
    
    tip_poses_2d_dt = pd.to_datetime(np.array(tip_poses_2d)[:,0].tolist(), unit='s')    
    tip_poses_2d = pd.DataFrame(np.array(tip_poses_2d)[:,1:3].tolist(), index=tip_poses_2d_dt)
    tip_poses_2d_resampled = tip_poses_2d.resample('10ms', how='mean')
    tip_poses_2d_interp = tip_poses_2d_resampled.interpolate()
    tip_poses_2d_interp_list = tip_poses_2d_interp.values.tolist()
    
    object_poses_2d_dt = pd.to_datetime(np.array(object_poses_2d)[:,0].tolist(), unit='s')
    object_poses_2d = pd.DataFrame(np.array(object_poses_2d)[:,1:3].tolist(), index=object_poses_2d_dt)
    object_poses_2d_resampled = object_poses_2d.resample('10ms', how='mean')
    object_poses_2d_interp = object_poses_2d_resampled.interpolate()
    object_poses_2d_interp_list = object_poses_2d_interp.values.tolist()
    
    force_dt = pd.to_datetime(np.array(force_2d)[:,0].tolist(), unit='s')
    force_2d = pd.DataFrame(np.array(force_2d)[:,1:3].tolist(), index=force_dt)
    force_2d_resampled = force_2d.resample('10ms', how='mean')
    force_2d_interp = force_2d_resampled.interpolate()
    force_2d_interp_list = force_2d_interp.values.tolist()
    
    data_resample = {}
    data_resample['tip_poses_2d'] = tip_poses_2d_interp_list
    data_resample['object_poses_2d'] = object_poses_2d_interp_list
    data_resample['force_2d'] = force_2d_interp_list
    return data_resample
    
def main(argv):
    if len(argv) < 2:
        print 'Usage: sync_json.py *.json'
        return
        
    bag_filepath = argv[1]
    with open(bag_filepath) as data_file:    
        data = json.load(data_file)
    
    shape_id = 'rect1'
    data2d = extract2d_and_cleanup(data)
    data_synced = resample_using_pandas(data2d)
    


if __name__=='__main__':
    import sys
    main(sys.argv)
    
