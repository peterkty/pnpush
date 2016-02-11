#!/usr/bin/env python


import subprocess
import sys, os
import glob
import optparse
import matplotlib.pyplot as plt
import h5py
import config.helper as helper
import numpy as np

def main(argv):
    parser = optparse.OptionParser()
    
    parser.add_option('', '--fmt', action="store", dest='fmt', 
                      help='Figure format e.g. png, pdf', default='png')
  
    (opt, args) = parser.parse_args()
    
    # files = ['/home/mcube/pnpushdata/friction_scan/plywood/rect1/record_surface=plywood_shape=rect1_a=-1000_v=20_rep=%03d_fcot.h5',
             # '/home/mcube/pnpushdata/friction_scan/delrin/rect1/record_surface=delrin_shape=rect1_a=-1000_v=20_rep=%03d_fcot.h5',
             # '/home/mcube/pnpushdata/friction_scan/pu/rect1/record_surface=pu_shape=rect1_a=0_v=20_rep=%03d_fcot.h5',
             # '/home/mcube/pnpushdata/friction_scan/abs/rect1/record_surface=abs_shape=rect1_a=0_v=20_rep=%03d_fcot.h5']
    files = ['/home/mcube/pnpushdata/friction_scan/plywood/rect1/record_surface=plywood_shape=rect1_a=-1000_v=20_rep=%03d_fcot.h5',
             '/home/mcube/pnpushdata/friction_scan/delrin/rect1/record_surface=delrin_shape=rect1_a=-1000_v=20_rep=%03d_fcot.h5',
             '/home/mcube/pnpushdata/friction_scan/pu/rect1/record_surface=pu_shape=rect1_a=0_v=20_rep=%03d_fcot.h5',
             '/home/mcube/pnpushdata/friction_scan/abs/rect1/record_surface=abs_shape=rect1_a=0_v=20_rep=%03d_fcot.h5']
             
    
    legends = ['plywood',
             'delrin',
             'pu',
             'abs']
             
    linestyles = ['-', '--', '-.', ':']
    for ind, filename in enumerate(files):
        if not os.path.isfile(filename):
            continue
        print ind, filename
        with h5py.File(filename, "r") as f:
            image_avgs_t = f['image_avgs_t'].value
            image_stds_t = f['image_stds_t'].value
        tmp = []
        for t in range(len(image_avgs_t)):
            tmp.append(0)
            for idx in range(len(image_avgs_t[t])):
                for idy in range(len(image_avgs_t[t][idx])):
                    tmp[-1] += image_avgs_t[t][idx][idy] / 9.0
        plt.errorbar(range(1,len(image_avgs_t)+1), tmp, fmt=linestyles[ind], label=legends[ind], color='k')
        print legends[ind], 'start', tmp[0], 'end', tmp[-1], 'chanage', (tmp[-1]-tmp[0]) / tmp[0]
    plt.legend()
    axes = plt.gca()
    axes.set_ylim([0.1, 0.37])
    
    plt.ylabel('Coefficient of friction')
    plt.xlabel('Push time')
    #plt.title('Change of frictional coefficient over time')
    plt.savefig('/home/mcube/pnpushdata/friction_scan/friction_overtime_overmaterial_lineplot.png')
    plt.savefig('/home/mcube/pnpushdata/friction_scan/friction_overtime_overmaterial_lineplot.pdf')
    
    plt.show()
    
    
if __name__=='__main__':
    main(sys.argv)
    
