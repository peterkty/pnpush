#!/usr/bin/env python

# Peter KT Yu, Aug 2015
# main function to parse a folder of bagfiles 

import subprocess
import sys, os

if __name__=='__main__':
    import glob
    dirname = sys.argv[1]
    #filelist = glob.glob("/home/mcube/pnpushdata/push_dataset_motion_soso/*.bag")
    filelist = glob.glob("%s/*.bag" % dirname)
    
    for bag_filepath in filelist:
        
        #if not os.path.exists(bag_filepath.replace('bag','json')):
        proc = subprocess.Popen('rosrun pnpush_planning parse_bagfile_to_rawjson_notf.py %s' % (bag_filepath) , shell=True)
        proc.wait()
            
        #if not os.path.exists(bag_filepath.replace('bag','png')):
        proc = subprocess.Popen('rosrun pnpush_planning plot_raw_json_moreshape.py %s' % (bag_filepath.replace('.bag', '.json')) , shell=True)
        proc.wait()
