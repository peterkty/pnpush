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
from ik.helper import *
from matplotlib.pyplot import savefig
import time

from mpl_toolkits.mplot3d import Axes3D


def plot_training_data(data, indexes, labels, title, xticks = None, 
    yticks = None, axisequal = True, figfname = None, bottom = None, left = None,
    xlim = None, ylim = None):
    data = np.array(data)
    print '[plot_training_data]', data.shape
    
    fig = plt.figure(figsize=(5,4))
    #fig.set_size_inches(7,7)
    plt.rc('text', usetex=True)
    
    plt.rc('font', family='san-serif', size=20)
    plt.title(title)
    
    if len(indexes) == 3:
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(data[:,indexes[0]], data[:,indexes[1]], data[:,indexes[2]], s=5, marker='.')
    else:
        ax = fig.add_subplot(111)
        ax.scatter(data[:,indexes[0]], data[:,indexes[1]], marker='.', s=5)
    
    
    
    ax.set_xlabel(labels[indexes[0]])
    ax.set_ylabel(labels[indexes[1]])
    if len(indexes) == 3:
        ax.set_zlabel(labels[indexes[2]])
    if axisequal:
        plt.axis('equal')
        
    if xlim:
        ax.set_xlim(xlim)
    if ylim:
        ax.set_ylim(ylim)
        
    xticks = plt.xticks()
    yticks = plt.yticks()
    plt.xticks(np.linspace(xticks[0][1], -xticks[0][1], 3))
    plt.yticks(np.linspace(yticks[0][1], -yticks[0][1], 3))
    
    fig.subplots_adjust(left=left, bottom=bottom, right=None, top=None,
                    wspace=None, hspace=None)
    if figfname:
        plt.savefig(figfname)
    else:
        plt.show()
    
def plot_limit_surface(data, xticks = None, 
    yticks = None, axisequal = True, figfname = None, bottom = None, left = None,
    xlim = None, ylim = None):
    data = np.array(data)
    print '[plot_limit_surface]', data.shape
    
    fig = plt.figure(figsize=(8,6.5))
    plt.rc('text', usetex=True)
    
    plt.rc('font', family='serif', size=30)
    
    fm = data[:,0]*data[:,5] - data[:,1]*data[:,4]
        
    ax = fig.add_subplot(111)
    ax.scatter(data[:,4], fm[:], marker='.')
    #ax.scatter(data[:,4], data[:,5], marker='.')
    
    if xlim:
        ax.set_xlim(xlim)
    if ylim:
        ax.set_ylim(ylim)
    
    xticks = plt.xticks()
    yticks = plt.yticks()
    plt.xticks(np.linspace(xticks[0][1], -xticks[0][1], 3))
    plt.yticks(np.linspace(yticks[0][1], -yticks[0][1], 3))
    
    ax.set_xlabel('$f_x$')
    ax.set_ylabel('$f_m$')
    fig.subplots_adjust(left=left, bottom=bottom, right=None, top=None,
                    wspace=None, hspace=None)
    if axisequal:
        plt.axis('equal')
    if figfname:
        plt.savefig(figfname)
    else:
        plt.show()

def main(argv):
    inputfile= argv[1]
    with open(inputfile) as data_file:    
        all_training_data = json.load(data_file)

    tag = 'dynamic'
    outdirname = '/home/mcube/pn-icra16-pushdata/pn-icra16-draft/figures/'
    labels = ['$x$', '$y$', '$\Delta x$', '$\Delta y$', 'force $x$', 'force $y$', r'$\Delta x$', r'$\Delta y$', r'$\Delta \theta$']
    # plot_training_data(all_training_data, [0,1], labels, '', 
        # axisequal = True, figfname = outdirname+'motion_tippos_%s.pdf' % tag,
        # bottom = 0.16, left = 0.22)
    # plot_training_data(all_training_data, [2,3], labels, '', 
        # axisequal = True, figfname = outdirname+'motion_tipdpos_%s.pdf' % tag,
        # bottom = 0.16, left = 0.22)
    # plot_training_data(all_training_data, [4,5], labels, '', 
        # axisequal = True, figfname = outdirname+'motion_force_%s.pdf' % tag,
        # bottom = 0.16, left = 0.22, xlim = [-4, 4], ylim = [-4, 4])
    # plot_training_data(all_training_data, [6,7], labels, '', 
        # axisequal = False, figfname = outdirname+'motion_objectdpose_xy_%s.pdf' % tag,
        # bottom = 0.16, left = 0.22)
    # plot_training_data(all_training_data, [6,8], labels, '', 
        # axisequal = False, figfname = outdirname+'motion_objectdpose_xt_%s.pdf' % tag,
        # bottom = 0.16, left = 0.22, xlim = [-0.015, 0.015], ylim = [-0.2, 0.2])
    
    # plot_limit_surface(all_training_data, axisequal = False, 
        # figfname = outdirname+'motion_limitsurface_%s.pdf' % tag, bottom = 0.16, left = 0.22,
        # xlim = [-8, 8], ylim = [-0.4, 0.4])

if __name__=='__main__':
    import sys
    main(sys.argv)
