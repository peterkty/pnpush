#!/usr/bin/env python

# Peter KT Yu, Feb 2016
# use the cage finger to push rect1 to scan the friction of the surface.


import optparse
import rospy
import numpy as np
from config.surface_db import SurfaceDB
from config.helper import pause
import config.helper as helper
import ik.helper
import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *
roslib.load_manifest("netft_rdt_driver")
from netft_rdt_driver.srv import Zero
import os
import tf.transformations as tfm


setCartRos = rospy.ServiceProxy('/robot2_SetCartesian', robot_SetCartesian)
setZero = rospy.ServiceProxy('/zero', Zero)
setAcc = rospy.ServiceProxy('/robot2_SetAcc', robot_SetAcc)
setZone = rospy.ServiceProxy('/robot2_SetZone', robot_SetZone)
setSpeed = rospy.ServiceProxy('/robot2_SetSpeed', robot_SetSpeed)


def wait_for_ft_calib():
    from ik.roshelper import ROS_Wait_For_Msg
    ROS_Wait_For_Msg('/netft_data', geometry_msgs.msg.WrenchStamped).getmsg()
    
def setCart(pos, ori):
    
    param = (np.array(pos) * 1000).tolist() + ori
    print 'setCart', param
    #pause()
    setCartRos(*param)
    
def main(argv):
    rospy.init_node('collect_friction_map_data')
    
    parser = optparse.OptionParser()
    parser.add_option('', '--surface', action="store", dest='surface_id', 
                      help='The surface id e.g. plywood, abs', default='plywood')
                      
    parser.add_option('', '--cage', action="store_true", dest='tocage', 
                      help='Do the cage process (placing the object inside the cage finger) or not. ', default=False)
    (opt, args) = parser.parse_args()
    
    shape_id = 'rect1'
    ori = [0, 0, 1, 0]  # qw qx qy qz
    center_world = [0.375, 0, 0]
    z = 0.147-0.01158 + SurfaceDB().db[opt.surface_id]['thickness']
    z_place = z+0.03
    z_up = z_place+0.03
    globalmaxacc = 100
    topics = ['-a']
    skip_when_exists = True
    
    global_speed = 100
    global_slow_speed = 20
    
    
    setZone(0)
    # scanning
    
    resolution = 0.005
    eps = 1e-6
    max_x = 0.450
    min_x = 0.250
    max_y = 0.197
    min_y = -0.233
    center = [(max_x + min_x) /2, (max_y + min_y) /2 ]
    range_x = np.linspace(max_x, min_x, 3)
    acc = 0
    vel = 20
    degs = xrange(0, 360, 5)
    rep = 0
    #radius = 0.06
    radii = [0.06, 0.03, 0]
    rotdegs_default = np.linspace(0, 300, 11)
     
    dir_save_bagfile = os.environ['PNPUSHDATA_BASE'] + '/friction_scan_limitsurface/%s/%s/' % (opt.surface_id,shape_id)
    helper.make_sure_path_exists(dir_save_bagfile)
    
    rep = 0
            
    for radius in radii:
        for deg in degs_order:  # translation velocity direction
            th = np.deg2rad(deg)
            if radius == 0:
                rotdegs = [10]
                setSpeed(tcp=vel, ori=10) # to prevent rotating very quick
            else:
                rotdegs = rotdegs_default
                setSpeed(tcp=vel, ori=1000)
                
            for rotdeg in rotdegs:  # rotation velocity direction
                rotth = np.deg2rad(deg)
                start_ori = ik.helper.qwxyz_from_qxyzw(tfm.quaternion_from_matrix((np.dot(tfm.euler_matrix(0,np.pi,0), tfm.euler_matrix(0,0,rotth)))))
                end_ori = ik.helper.qwxyz_from_qxyzw(tfm.quaternion_from_matrix((np.dot(tfm.euler_matrix(0,np.pi,0), tfm.euler_matrix(0,0,-rotth)))))
                start_pos = [np.cos(th)* radius + center[0], np.sin(th)* radius + center[1]]
                end_pos = [np.cos(th+np.pi)* radius + center[0], np.sin(th+np.pi)* radius + center[1]]
            
                bagfilename = 'record_surface=%s_shape=%s_a=%.0f_v=%.0f_deg=%d_rotdeg=%d_radius=%.2f_rep=%03d.bag' % (opt.surface_id, shape_id, acc*1000, vel, deg, rotdeg, radius, rep)
                print bagfilename
                bagfilepath = dir_save_bagfile+bagfilename
                
                if skip_when_exists and os.path.isfile(bagfilepath):
                    #print bagfilepath, 'exits', 'skip'
                    continue  
                
                setSpeed(tcp=global_slow_speed, ori=1000)
                setCart([start_pos[0], start_pos[1], z], start_ori)
                setCart([start_pos[0], start_pos[1], z_place], start_ori)
                setZero()
                wait_for_ft_calib()
                setCart([start_pos[0], start_pos[1], z], start_ori)
                setSpeed(tcp=vel, ori=1000)

                rosbag_proc = helper.start_ros_bag(bagfilename, topics, dir_save_bagfile)
                setCart([end_pos[0], end_pos[1], z], end_ori)

                helper.terminate_ros_node("/record")
            
    rospy.sleep(1)   # make sure record is terminated completely
    
if __name__=='__main__':
    main(sys.argv)
    

