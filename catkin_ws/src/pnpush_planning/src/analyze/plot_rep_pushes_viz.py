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
from ik.helper import *
from config.helper import *
from config.shape_db import ShapeDB
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection

def plot_point_cov(points, nstd=2, ax=None, **kwargs):
    """
    Plots an `nstd` sigma ellipse based on the mean and covariance of a point
    "cloud" (points, an Nx2 array).

    Parameters
    ----------
        points : An Nx2 array of the data points.
        nstd : The radius of the ellipse in numbers of standard deviations.
            Defaults to 2 standard deviations.
        ax : The axis that the ellipse will be plotted on. Defaults to the 
            current axis.
        Additional keyword arguments are pass on to the ellipse patch.

    Returns
    -------
        A matplotlib ellipse artist
    """
    pos = points.mean(axis=0)
    cov = np.cov(points, rowvar=False)
    return plot_cov_ellipse(cov, pos, nstd, ax, **kwargs)

def plot_cov_ellipse(cov, pos, nstd=2, ax=None, **kwargs):
    """
    Plots an `nstd` sigma error ellipse based on the specified covariance
    matrix (`cov`). Additional keyword arguments are passed on to the 
    ellipse patch artist.

    Parameters
    ----------
        cov : The 2x2 covariance matrix to base the ellipse on
        pos : The location of the center of the ellipse. Expects a 2-element
            sequence of [x0, y0].
        nstd : The radius of the ellipse in numbers of standard deviations.
            Defaults to 2 standard deviations.
        ax : The axis that the ellipse will be plotted on. Defaults to the 
            current axis.
        Additional keyword arguments are pass on to the ellipse patch.

    Returns
    -------
        A matplotlib ellipse artist
    """
    def eigsorted(cov):
        vals, vecs = np.linalg.eigh(cov)
        order = vals.argsort()[::-1]
        return vals[order], vecs[:,order]

    if ax is None:
        ax = plt.gca()

    vals, vecs = eigsorted(cov)
    theta = np.degrees(np.arctan2(*vecs[:,0][::-1]))

    # Width and height are "full" widths, not radius
    width, height = 2 * nstd * np.sqrt(vals)
    ellip = mpatches.Ellipse(xy=pos, width=width, height=height, angle=theta, **kwargs)

    ax.add_artist(ellip)
    return ellip

def main(argv):
    parser = optparse.OptionParser()
    
    parser.add_option('-s', action="store", dest='shape_id', 
                      help='The shape id e.g. rect1-rect3', default='rect1')
                      
    parser.add_option('', '--surface', action="store", dest='surface_id', 
                      help='The surface id e.g. plywood, abs', default='plywood')
                      
    parser.add_option('', '--nrep', action="store", dest='nrep', type='int',
                      help='Repeat how many times', 
                      default=5000)  
    parser.add_option('', '--reptype', action="store", dest='reptype', type='string',
                      help='Repeat type', 
                      default='normal')  
    
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
    
    figfname_png = dir_to_rep_push + '/rep_push_viz_%s.png' % opt.surface_id
    figfname_pdf = dir_to_rep_push + '/rep_push_viz_%s.pdf' % opt.surface_id
    
    figfname_hist_png = dir_to_rep_push + '/rep_push_viz_hist_%s.png' % opt.surface_id
    figfname_hist_pdf = dir_to_rep_push + '/rep_push_viz_hist_%s.pdf' % opt.surface_id
    
    cachefile = '/tmp/plot_rep_push'
    import shelve
    if os.path.exists(cachefile):
        f = shelve.open(cachefile)
        vals = f['vals'];
        trajs = f['trajs'];
        fts = f['fts']
        opt = f['opt']
        trajs_tippose = f['trajs_tippose']
        meantraj = f['meantraj']
        meantraj_tippose = f['meantraj_tippose']
    else:
        vals = [] # delta between start and end
        trajs = []
        trajs_tippose = []
        fts = []
        for rep in xrange(opt.nrep):
            h5filename = 'motion_surface=%s_shape=%s_a=%.0f_v=%.0f_i=%.3f_s=%.3f_t=%.3f_rep=%04d.h5' % (opt.surface_id, opt.shape_id, acc*1000, vel, i, s, t, rep)
            
            #filename = 'motion_surface=%s_shape=%s_a=%.0f_v=%.0f_i=%.3f_s=%.3f_t=%.3f_rep=%04d.bag' % (opt.surface_id, opt.shape_id, acc*1000, speeds[cnt_acc], i, s, t, rep)
            filepath = '%s/%s/%s/%s/%s' % (dir_to_rep_push,opt.surface_id,opt.shape_id,opt.reptype,h5filename)
            if not os.path.isfile(filepath):
                print 'not exists', filepath
                break
            
            f = h5py.File(filepath, "r")
            ft_array = f['ft_wrench'].value
            object_pose = f['object_pose'].value
            tip_pose = f['tip_pose'].value
            f.close()
            
            invT0 = np.linalg.inv(matrix_from_xyzrpy(object_pose[0][1:3].tolist() + [0], [0,0,object_pose[0][3]]))
            
            T = matrix_from_xyzrpy(object_pose[-1][1:3].tolist() + [0], [0,0,object_pose[-1][3]])
            T_T0 = np.dot(invT0, T)
            scale, shear, angles, trans, persp = tfm.decompose_matrix(T_T0)
            vals.append(np.append(trans[0:2] ,np.unwrap([angles[2]])))
            
            # extract traj
            if rep in xrange(500):
            #if True:
                traj = []
                for p in object_pose:
                    T = matrix_from_xyzrpy(p[1:3].tolist() + [0], [0,0,p[3]])
                    T_T0 = np.dot(invT0, T)
                    scale, shear, angles, trans, persp = tfm.decompose_matrix(T_T0)
                    traj.append(np.append([p[0]-object_pose[0][0]], np.append(trans[0:2] , np.unwrap([angles[2]]))) )
                trajs.append(traj)
                
                traj_tippose = []
                for tip_pose_ in tip_pose:
                    traj_tippose.append(np.append([tip_pose_[0]-tip_pose[0][0]], np.dot(invT0, tip_pose_[1:3].tolist()+[0,1])))
                trajs_tippose.append(traj_tippose)
            
    
        def computeMeanTraj(trajs):
            lenn = 1000000
            for traj in trajs:
                lenn = min(lenn, len(traj))
            
            ncol = len(trajs[0][0])
            meantraj = np.zeros((lenn, ncol))
            ntraj = len(trajs)
            for traj in trajs:
                meantraj = meantraj + np.array(traj[0:lenn]) / ntraj
                
            return meantraj
            
        meantraj = computeMeanTraj(trajs)
        meantraj_tippose = computeMeanTraj(trajs_tippose)
        
        
        ll = locals()
        shv = shelve.open(cachefile, 'n')
        for key, val in ll.iteritems():
            try:
                shv[key] = val
            except:
                pass
        shv.close()
    
    (x,y,th)=zip(*(vals))
    
    valscov = np.cov(vals, rowvar=0)
    valsmean = np.mean(vals, axis=0)
    print 'covariance\n', valscov
    print 'mean', valsmean
    
    
    #from latexify import latexify; latexify(fig_width=3.39, fig_height=3.39*(sqrt(5)-1.0)/2.0*2,scale = 2)
    from latexify import latexify; latexify(scale = 2)
    

    
    
    
        
    f1, ((ax1, ax2, ax3)) = plt.subplots(1, 3, sharex=False, sharey=False)
    #fig = plt.figure()
    # ax1 = fig.add_subplot(2, 1, 1)
    # ax2 = fig.add_subplot(2, 2, 1, sharex=ax1, sharey=ax1)
    plt.sca(ax1)
    ax = ax1
    (tm, x,y,th) = zip(*meantraj)
    line = plt.plot(x, y, '-k')
    plt.setp(line, linewidth=2)
    
    #### add the object as polygon
    shape_db = ShapeDB()
    shape = shape_db.shape_db[opt.shape_id]['shape'] # shape of the objects presented as polygon.
    shape_type = shape_db.shape_db[opt.shape_id]['shape_type']
    if shape_type == 'poly':
        shape_polygon_3d = np.hstack((np.array(shape), np.zeros((len(shape), 1)), np.ones((len(shape), 1))))
    elif shape_type == 'ellip':
        shape = shape[0]
    elif shape_type == 'polyapprox':
        shape_polygon_3d = np.hstack((np.array(shape[0]), np.zeros((len(shape[0]), 1)), np.ones((len(shape[0]), 1))))
        
    ec, fc = 'black','orangered'
    for ii in np.linspace(0, len(meantraj)-1, 30):
        i = int(ii)
        
        if i == 0:
            alpha , fill = (0.3, True)
        elif i == len(meantraj)-1:
            alpha , fill = (0.6, True)
        else:
            alpha , fill = (0.6, False)
            
        T = matrix_from_xyzrpy([x[i], y[i], 0], [0, 0, th[i]])
        shape_polygon_3d_world = np.dot(T, shape_polygon_3d.T)
        obj = mpatches.Polygon(shape_polygon_3d_world.T[:,0:2], closed=True, fc=fc, ec=ec, alpha=alpha, fill=fill, linewidth=1, linestyle='solid')
        
        ax.add_patch(obj)
    #####
    
    # add the probes as circle
    probe_radius = 0.00475
    for ind, ii in enumerate(np.linspace(0, len(meantraj_tippose)-1, 30)):
        if ind <= 3: continue
        i = int(ii)
        if i == 0:
            alpha , fill = (0.8, False)
        elif i == len(meantraj_tippose)-1:
            alpha , fill = (0.8, False)
        else:
            alpha , fill = (0.8, False)
        circle = mpatches.Circle(meantraj_tippose[i][1:3], probe_radius, color='black', alpha=alpha, fill=fill, linewidth=1, linestyle='solid')
            
        ax.add_patch(circle)
    
    
    plt.axis('equal') 
    plt.axis('off')
    
    # 2. plot all traj
    ax = ax2
    plt.sca(ax)
    
    for traj in trajs:
        (tm, x,y,th) = zip(*traj)
        plt.plot(x, y, 'b', alpha=0.5)
        
    #   plot begin and final mean block
    (tm,x,y,th) = zip(*meantraj)
    for i in [0, -1]:
        alpha , fill = (0.6, False)
        T = matrix_from_xyzrpy([x[i], y[i], 0], [0, 0, th[i]])
        shape_polygon_3d_world = np.dot(T, shape_polygon_3d.T)
        obj = mpatches.Polygon(shape_polygon_3d_world.T[:,0:2], closed=True, fc=fc, ec=ec, alpha=alpha, fill=fill, linewidth=1, linestyle='solid',  zorder=2)
        ax.add_patch(obj)
    
    #line = plt.plot(x, y, '-k')
    
    plot_cov_ellipse(valscov[0:2][:,0:2], valsmean[0:2], color='orangered', fill=True, alpha=0.9,  zorder=3)
    plt.setp(line, linewidth=2)
    plt.axis('equal') 
    plt.axis('off')
    
    # 3. plot final poses
    ax = ax3
    plt.sca(ax)
    (xd,yd,thd)=zip(*(vals))
    ax.scatter(xd,yd, s=0.2, color='k', alpha=1)
    
    #   plot begin and final mean block
    (tm,x,y,th) = zip(*meantraj)
    for i in [0,-1]:
        alpha , fill = (0.6, False)
        T = matrix_from_xyzrpy([x[i], y[i], 0], [0, 0, th[i]])
        shape_polygon_3d_world = np.dot(T, shape_polygon_3d.T)
        #obj = mpatches.Polygon(shape_polygon_3d_world.T[:,0:2], closed=True, fc=fc, ec=ec, alpha=alpha, fill=fill, linewidth=1, linestyle='solid')
        #ax.add_patch(obj)
    
    # plot 2 sigma bound
    
    plot_cov_ellipse(valscov[0:2][:,0:2], valsmean[0:2], color='orangered', fill=True, alpha=0.9,  zorder=0)
    #plot_cov_ellipse(valscov[0:2][:,0:2], valsmean[0:2], 3, color='orangered', fill=True, alpha=0.5,  zorder=0)
    #ax.add_patch(obj)
        
    plt.axis('equal') 
    plt.axis('off')
    #ax2.set_title('Scatter plot: $\Delta x$ versus $\Delta y$')
        
    plt.tight_layout(pad=0, w_pad=0, h_pad=0)
    plt.subplots_adjust(left=0.08, bottom=0.06, right=0.97, top=1.0,
                wspace=0.01, hspace=0.20)
    plt.savefig(figfname_png, dpi=200)
    plt.savefig(figfname_pdf)
    plt.show()
    
    #   plot histogram
    
    f2, ((ax4, ax5, ax6)) = plt.subplots(1, 3, sharex=False, sharey=False)
    plt.sca(ax4)
    plt.locator_params(axis='x',nbins=4)
    n, bins, patches= ax4.hist(np.array(xd)*1000, 200, normed=1, histtype='stepfilled', 
         facecolor='none', label='x', alpha=1)
    ax4.get_yaxis().set_visible(False)
    ax4.set_xlabel('$\Delta x$ (mm)')
    #ax4.set_title('Histogram of $\Delta x$')
    
    plt.sca(ax5)
    plt.locator_params(axis='x',nbins=4)
    n, bins, patches= ax5.hist(np.array(yd)*1000, 200, normed=1, histtype='stepfilled', 
         facecolor='none', label='y', alpha=1)
    ax5.get_yaxis().set_visible(False)
    ax5.set_xlabel('$\Delta y$ (mm)')
    #ax5.set_title('Histogram of $\Delta y$')
    
    plt.sca(ax6)
    plt.locator_params(axis='x',nbins=4)
    n, bins, patches= ax6.hist(np.rad2deg(thd), 200, normed=1, histtype='stepfilled', 
         facecolor='none', label='theta', alpha=1)
    ax6.get_yaxis().set_visible(False)
    ax6.set_xlabel('$\Delta \\theta$ (degree)')
    #ax6.set_title('Histogram of $\Delta \\theta$')
        
    plt.tight_layout(pad=0, w_pad=0, h_pad=0)
    plt.subplots_adjust(left=0.04, bottom=0.23, right=0.96, top=0.87,
                wspace=0.22, hspace=0.20)
    plt.savefig(figfname_hist_png, dpi=200)
    plt.savefig(figfname_hist_pdf)
    plt.show()
    
        
if __name__=='__main__':
    import sys
    main(sys.argv)
