#!/usr/bin/env python

# Peter KT Yu, Aug 2015
# parse the bagfiles into simple format with timestamp: probe pose, object pose and ft wrench

import rosbag
import time # for sleep
#import rospy
import json
from ik.helper import *
from sensor_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import tf
import tf.transformations as tfm
import h5py

import sys
import subprocess

useRRI = True
useH5 = True

def ftmsg2listandflip(ftmsg):
    return [-ftmsg.wrench.force.x,ftmsg.wrench.force.y,-ftmsg.wrench.force.z,
            -ftmsg.wrench.torque.x,ftmsg.wrench.torque.y,-ftmsg.wrench.torque.z]
            # the sign flip is for transforming to robot frame
            
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
        print 'Also make sure roscore is running'
        return
    
        
    bag_filepath = argv[1]
    json_filepath = bag_filepath.replace('.bag', '.json')
    hdf5_filepath = bag_filepath.replace('.bag', '.h5')
    print 'bag_filepath:', bag_filepath
    print 'json_filepath:', json_filepath
    print 'hdf5_filepath:', hdf5_filepath
    
    
    
    
    import rosbag
    bag = rosbag.Bag(bag_filepath)
    
    vicon_to_world = []
    for topic, msg, t in bag.read_messages(topics=['/tf']):
        for i in range(len(msg.transforms)):
            frame_id = msg.transforms[i].header.frame_id
            child_frame_id = msg.transforms[i].child_frame_id
            t = msg.transforms[i].transform
            if child_frame_id == '/viconworld':
                vicon_to_world = [t.translation.x,t.translation.y,t.translation.z] + [t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]
                break
        if len(vicon_to_world) > 0: break
                 
    
    T_viconworld_to_world = poselist2mat(vicon_to_world)
    
    for topic, msg, t in bag.read_messages(topics=['/vicon/StainlessSteel/StainlessSteel']):
        pose_viconworld = [msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z,
            msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w]
        
        T_object_to_viconworld = poselist2mat(pose_viconworld)
        
        T_object_to_world = np.dot(T_viconworld_to_world, T_object_to_viconworld)
        
        object_to_world = mat2poselist(T_object_to_world)
        
        object_pose_array.append([msg.header.stamp.to_sec()] + object_to_world)
    
    if useRRI:
        for topic, msg, t in bag.read_messages(topics=['/robot2_RRICartState']):
            tip_array.append([msg.header.stamp.to_sec(), 
                msg.position[0]/1000,msg.position[1]/1000])
    else:
        for topic, msg, t in bag.read_messages(topics=['/robot2_CartesianLog']):
            tip_array.append([msg.timeStamp, 
                msg.x/1000,msg.y/1000,msg.z/1000,
                msg.qx,msg.qy,msg.qz,msg.q0])
        
    for topic, msg, t in bag.read_messages(topics=['/netft_data']):
        ft_array.append([msg.header.stamp.to_sec()] + ftmsg2listandflip(msg))
        
        
    bag.close()
    
    
    data = {'tip_poses': tip_array, 'ft_wrench': ft_array, 'object_pose': object_pose_array}
    
    # save the data
    with open(json_filepath, 'w') as outfile:
        json.dump(data, outfile, indent=4)
        
    
    # save the data as hdf5
    with h5py.File(hdf5_filepath, "w") as f:
        f.create_dataset("tip_array", data=tip_array)
        f.create_dataset("ft_wrench", data=ft_array)
        f.create_dataset("object_pose", data=object_pose_array)
    
if __name__=='__main__':
    main(sys.argv)

