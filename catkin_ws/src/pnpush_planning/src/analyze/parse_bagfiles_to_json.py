
import subprocess

if __name__=='__main__':
    import glob
    #filelist = glob.glob("/home/mcube/pnpushdata/push_dataset_motion_soso/*.bag")
    filelist = glob.glob("/home/mcube/pnpushdata/push_dataset_id/*.bag")
    
    for bag_filepath in filelist:
        
        # proc = subprocess.Popen('rosrun pnpush_planning parse_bagfile_to_json.py %s' % (bag_filepath) , shell=True)
        proc = subprocess.Popen('rosrun pnpush_planning parse_bagfile_to_json_notf.py %s' % (bag_filepath) , shell=True)
        # 
        # proc.wait()

        #proc = subprocess.Popen('rosrun pnpush_planning plot_json.py %s' % (bag_filepath.replace('.bag', '.json')) , shell=True)
        #proc = subprocess.Popen('rosrun pnpush_planning plot_json.py %s' % (bag_filepath.replace('.bag', '.json')) , shell=True)
        
        proc.wait()
