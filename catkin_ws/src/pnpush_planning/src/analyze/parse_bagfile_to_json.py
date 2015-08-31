#!/usr/bin/env python

# Peter KT Yu, Aug 2015
# parse the bagfiles into simple format: probe pose, object pose and ft wrench

import rosbag
import time # for sleep
import rospy
import json
from sensor_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import tf

def callback_tip(data):
    global listener
    global tip_array
    try:
        t = listener.getLatestCommonTime('link_6','base_link')
        (pos,ori) = listener.lookupTransform('base_link','link_6',t)
        tip_array.append([t.to_sec()] + list(pos) + list(ori))
    except:
        pass
        #print t

def callback_vicon(data):
    global listener
    global object_pose_array
    try:
        t = listener.getLatestCommonTime('/vicon/StainlessSteel/StainlessSteel','map')
        (pos,ori) = listener.lookupTransform('map','/vicon/StainlessSteel/StainlessSteel',t)
        object_pose_array.append([t.to_sec()] + list(pos) + list(ori))
    except:
        pass

def ftmsg2list(ftmsg):
    return [ftmsg.wrench.force.x,ftmsg.wrench.force.y,ftmsg.wrench.force.z,
            ftmsg.wrench.torque.x,ftmsg.wrench.torque.y,ftmsg.wrench.torque.z]

def callback_ft(data):
    global ft_array
    global pub_ft
    ft_array.append([data.header.stamp.to_sec()] + ftmsg2list(data))


import sys
import subprocess

def main(argv):
    global pub
    global pub_ft
    global listener
    global tip_array
    global object_pose_array
    global ft_array
    tip_array = []
    ft_array = []
    object_pose_array = []
    
    if len(argv) < 2:  # no bagfile name
        print 'Usage: parse_bagfile_to_json.py [bag_file_path.bag]'
        print 'Also make sure no roscore is running'
        return
    
        
    bag_filepath = argv[1]
    json_filepath = bag_filepath.replace('.bag', '.json')
    print 'bag_filepath:', bag_filepath
    print 'json_filepath:', json_filepath
    
    
    #roscore_proc = subprocess.Popen('roscore', shell=True)
    rospy.init_node('listener', anonymous=True)
    
    listener = tf.TransformListener()
    rospy.Subscriber("/joint_states", JointState, callback_tip)
    rospy.Subscriber("/netft_data", WrenchStamped, callback_ft)
    rospy.Subscriber("/vicon/StainlessSteel/StainlessSteel", TransformStamped, callback_vicon)
    
    
    rosbag_proc = subprocess.Popen('rosbag play -q %s' % (bag_filepath) , shell=True)
    
    # while ros_node_alive('/play'):
    # #while rosbag_proc.poll():
        # sleep(0.1)
    rosbag_proc.wait()
    
    print 'end bag file'
    
    data = {'tip_poses': tip_array, 'ft_wrench': ft_array, 'object_pose': object_pose_array}
    
    # save the data
    with open(json_filepath, 'w') as outfile:
        json.dump(data, outfile, indent=4)
        
    #roscore_proc.terminate()

def ros_node_alive(s):
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for str in list_output.split("\n"):
        if (str.startswith(s)):
            return True

    
if __name__=='__main__':
    main(sys.argv)

