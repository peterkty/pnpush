#!/usr/bin/env python

import optparse, sys
import h5py
import config.helper as helper
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import pdb
import glob, os
import ik.helper
import tf.transformations as tfm

def main(argv):
    parser = optparse.OptionParser()
    
    parser.add_option('-s', action="store", dest='shape_id', 
                      help='The shape id e.g. rect1-rect3', default='rect1')
                      
    parser.add_option('', '--surface', action="store", dest='surface_id', 
                      help='The surface id e.g. plywood, abs', default='plywood')
                      
    parser.add_option('', '--nrep', action="store", dest='nrep', type='int',
                      help='Repeat how many times', 
                      default=50)  
    
    (opt, args) = parser.parse_args()
    
    if len(args) < 1:  # no bagfile name
        parser.error("Usage: plot_rep_pushes.py [dir_to_rep_push] e.g. /home/mcube/pnpushdata/straight_push_rep")  
        return
    dir_to_rep_push = args[0]
    
    vel = 20
    acc = 0
    i = 0
    s = 0.7
    t = 0
                        
    for rep in opt.nrep:
        h5filename = 'motion_surface=%s_shape=%s_a=%.0f_v=%.0f_rep=%03d.h5' % (opt.surface_id, shape_id, acc*1000, vel, i, rep)
        # motion_surface=plywood_shape=rect1_a=0_v=20_i=0.000_s=0.700_t=0.000_rep=0000.bag
        
        filepath = '%s/%s/%s/%s' % (dir_to_rep_push,opt.surface_id,opt.shape_id,h5filename)
        if not os.path.isfile(filepath):
            print 'not exists', filepath
            break
            
        f = h5py.File(filepath, "r")
        tip_array = f['tip_array'].value
        tip_array = f['tip_array'].value
        
        f.close()
    
