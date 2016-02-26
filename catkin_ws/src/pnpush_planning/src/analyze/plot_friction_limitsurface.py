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
    
    parser.add_option('', '--res', action="store", type='float', dest='res', 
                      help='Resolution in deg', nargs=1, default=(5))
    parser.add_option('', '--rep', action="store", type='int', dest='rep', 
                      help='How many repetitions', nargs=1, default=(1))
    parser.add_option('', '--limits', action="store", type='float', dest='limits', 
                      help='Limits in meter', nargs=4, default=(0.250,0.450, -0.233, 0.197)) # [minx, maxx, miny, maxy]
    parser.add_option('', '--N', action="store", type='float', dest='N', 
                      help='Normal force', nargs=1, default=(0.8374 * 9.81)) # Normal force
    parser.add_option('', '--surface', action="store", type='string', dest='surface_id', 
                      help='surface', nargs=1, default='plywood') # Normal force
                      
    (opt, args) = parser.parse_args()
    
    if len(args) < 1:  # no bagfile name
        parser.error("Usage: plot_friction_limitsurface.py [dir_to_friction_scan_limitsurface]")
        return
    
    dir_to_friction_scan_ls = args[0]
    
    figfname_png = dir_to_friction_scan_ls + '/friction_limitsurface_%s.png' % opt.surface_id
    figfname_pdf = dir_to_friction_scan_ls + '/friction_limitsurface_%s.pdf' % opt.surface_id
    ft_wrench = []
    
    min_x, max_x, min_y, max_y = opt.limits
    
    center = [(max_x + min_x) /2, (max_y + min_y) /2 ]
    acc = 0
    
    # a list of materials
    #dirlist = [ name for name in os.listdir(dir_to_friction_scan_iso) if os.path.isdir(os.path.join(dir_to_friction_scan_iso, name)) ]
    
    #dirlist = ['abs', 'delrin','plywood',  'pu']
    
    shape_id = 'rect1'
    vel = 20
    N = opt.N
    
    linestyles = [':', '-', '-', '-']
    markerstyles = ['', '', '^', 'o']
    markeverys = [1,1,3,3]
    rangex = xrange(0, 360, opt.res)
    raidus = 0.0005
    degs_default = xrange(0, 360, 5)
    rep = 0
    radii = [0, 0.05]
    rotdegs_default = np.linspace(0, 80, 21)
     
    
    
    vals = []
    for radius in radii:
        if radius == 0:
            degs = [0]
        else:
            degs = degs_default
        for deg in degs:  # translation velocity direction
            th = np.deg2rad(deg)
            
            if radius == 0:
                rotdegs = [10]
            else:
                rotdegs = rotdegs_default
                
            for rotdeg in rotdegs:  # rotation velocity direction
                rotth = np.deg2rad(deg)
                start_ori = ik.helper.qwxyz_from_qxyzw(tfm.quaternion_from_matrix((np.dot(tfm.euler_matrix(0,np.pi,0), tfm.euler_matrix(0,0,rotth)))))
                end_ori = ik.helper.qwxyz_from_qxyzw(tfm.quaternion_from_matrix((np.dot(tfm.euler_matrix(0,np.pi,0), tfm.euler_matrix(0,0,-rotth)))))
                start_pos = [np.cos(th)* radius + center[0], np.sin(th)* radius + center[1]]
                end_pos = [np.cos(th+np.pi)* radius + center[0], np.sin(th+np.pi)* radius + center[1]]
                
                vel_direc = [np.array(end_pos) - np.array(start_pos), 2*rotth]
            
                h5filename = 'record_surface=%s_shape=%s_a=%.0f_v=%.0f_deg=%d_rotdeg=%d_radius=%.3f_rep=%03d.h5' % (opt.surface_id, shape_id, acc*1000, vel, deg, rotdeg, radius, rep)
                
                filepath = '%s/%s/%s/%s' % (dir_to_friction_scan_ls,opt.surface_id,shape_id,h5filename)
                print 'processing', filepath
                if not os.path.isfile(filepath):
                    print 'not exists'
                    break
                    
                f = h5py.File(filepath, "r")
                tip_array = f['tip_pose'].value
                ft_wrench = f['ft_wrench'].value
                f.close()
                
                scale = (len(ft_wrench)*1.0/len(tip_array))
                
                for i, tip_pos  in enumerate(tip_array):
                    if np.linalg.norm(np.array(tip_pos[1:3]) - np.array(center)) < raidus:
                        ft_i = int(i * scale)
                        vals.append(list(ft_wrench[ft_i][1:3]) + list([ft_wrench[ft_i][3]]))  # force x y and torque in z
    
    
    from mpl_toolkits.mplot3d import Axes3D
    
    from latexify import latexify; latexify(scale = 2)
    
    fig = plt.figure()
    axes = fig.add_subplot(111, projection='3d')
    #axes = plt.gca()
    axes.grid(True, linewidth = 0.25, color='grey')
    axes.set_axisbelow(True)
    
    #import pdb; pdb.set_trace()
    #vals = vals[0::10]
    (x,y,z) = zip(*vals)
    axes.scatter(x, y, z, c=y, marker='.')
    
    axes.scatter(x, y, -np.array(z), c=y, marker='.')
        
    #plt.errorbar(rangex, diffs, color='k',  fmt=linestyles[inds]+markerstyles[inds], 
    # label=surface, markevery = markeverys[inds], markersize=5, linewidth=0.5)

    plt.tight_layout()
    axes.set_xlabel('force x')
    axes.set_ylabel('force y')
    axes.set_zlabel('moment')
        
    #legend = plt.legend(loc='lower right', ncol = 4)
    
    #legend.get_frame().set_linewidth(0.25)
    plt.subplots_adjust(left=0.12, bottom=0.13, right=None, top=None,
                wspace=None, hspace=None)
    plt.savefig(figfname_png)
    plt.savefig(figfname_pdf)
    plt.show()
    

if __name__=='__main__':
    main(sys.argv)
