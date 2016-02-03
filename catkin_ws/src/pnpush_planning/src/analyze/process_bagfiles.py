#!/usr/bin/env python

# Peter KT Yu, Aug 2015
# main function to parse a folder of bagfiles 

import subprocess
import sys, os
import glob

def main(argv):
    if len(sys.argv) < 2:
        print 'processs_bagfiles.py', 'dirname'
        return
        
    dirname = sys.argv[1]
    filelist = glob.glob("%s/*.bag" % dirname)
    
    toplot = True
    
    for bag_filepath in filelist:
        if not os.path.exists(bag_filepath.replace('bag','json')):
            proc = subprocess.Popen('rosrun pnpush_planning parse_bagfile_to_rawjson.py %s' % (bag_filepath) , shell=True)
            proc.wait()
            
        if toplot and not os.path.exists(bag_filepath.replace('bag','png')):
            proc = subprocess.Popen('rosrun pnpush_planning plot_raw_json.py %s snapshots' % (bag_filepath.replace('.bag', '.json')) , shell=True)
            proc.wait()


if __name__=='__main__':
    main(sys.argv)
    
