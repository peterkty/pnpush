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


def plot_training_data(data, indexes, labels, title):
    data = np.array(data)
    print '[plot_training_data]', data.shape
    
    fig = plt.figure(figsize=(5,4))
    #fig.set_size_inches(7,7)
    plt.rc('text', usetex=True)
    
    plt.rc('font', family='serif', size=20)
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
    plt.axis('equal')
    plt.xticks(np.linspace(-0.001, 0.001, 3))
    plt.yticks(np.linspace(-0.001, 0.001, 3))
    plt.show()
    
def plot_limit_surface(data):
    data = np.array(data)
    print '[plot_limit_surface]', data.shape
    
    fig = plt.figure(figsize=(8,6.5))
    plt.rc('text', usetex=True)
    
    plt.rc('font', family='serif', size=30)
    #plt.title('Limit surface')
    
    fm = data[:,0]*data[:,5] - data[:,1]*data[:,4]
    
    #ax = fig.add_subplot(111, projection='3d')
    #ax.scatter(data[:,4], data[:,5], fm[:], marker='.')
    
    ax = fig.add_subplot(111)
    #ax.scatter(data[:,4], fm[:], marker='.')
    ax.scatter(data[:,4], data[:,5], marker='.')
    
    
    ax.set_xlabel('$f_x$')
    #ax.set_ylabel('fy', fontsize=10)
    ax.set_ylabel('$f_y$')
    plt.xticks(np.linspace(-3, 3, 3))
    plt.yticks(np.linspace(-3, 3, 3))
    #plt.yticks(np.linspace(-0.15, 0.15, 3))
    plt.axis('equal')
    plt.show()

def main(argv):
    inputfile= "%s/data_training.json" % argv[1]
    with open(inputfile) as data_file:    
        all_training_data = json.load(data_file)

    labels = ['$x$', '$y$', '$v_x$', '$v_y$', 'force $x$', 'force $y$', r'$\Delta x$', r'$\Delta y$', r'$\Delta \theta$']
    #plot_training_data(all_training_data, [0,1], labels, '')
    # plot_training_data(all_training_data, [2,3], labels, '')
    # plot_training_data(all_training_data, [4,5], labels, '')
    #plot_training_data(all_training_data, [6,7], labels, '')
    
    plot_limit_surface(all_training_data)

if __name__=='__main__':
    import sys
    main(sys.argv)
