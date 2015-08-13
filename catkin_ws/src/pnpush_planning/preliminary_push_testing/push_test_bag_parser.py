

import rosbag
import os

savedPath = os.getcwd()
os.chdir(savedPath+'/St_block')

for numBags in range(1,7):
    pushTestNum = numBags
    bagFileNameStr = 'Test_st_%s.bag' % pushTestNum
    bag = rosbag.Bag(bagFileNameStr)

    # robot tcp x,y
    txtFileNameStr = 'metal_%s_tcp.txt' % pushTestNum
    text_file = open(txtFileNameStr, "w")

    for topic, msg, t in bag.read_messages(topics=['/tip_pose']):
        # print msg
        text_file.write('%s \n' % msg.header.stamp)
        text_file.write('%s \n' % msg.pose.position.x)
        text_file.write('%s \n' % msg.pose.position.y)
    text_file.close()


    # object pose x,y,z,theta_x,theta_y,theta_z w.r.t. world frame
    txtFileNameStr = 'metal_%s_obj.txt' % pushTestNum
    text_file = open(txtFileNameStr, "w")

    for topic, msg, t in bag.read_messages(topics=['/vicon/SteelBlock/SteelBlock']):
        # print msg
        text_file.write('%s \n' % msg.header.stamp)
        text_file.write('%s \n' % msg.transform.translation.x)
        text_file.write('%s \n' % msg.transform.translation.y)
        text_file.write('%s \n' % msg.transform.translation.z)
        text_file.write('%s \n' % msg.transform.rotation.x)
        text_file.write('%s \n' % msg.transform.rotation.y)
        text_file.write('%s \n' % msg.transform.rotation.z)
    text_file.close()

    # FT force x, y w.r.t. world frame
    txtFileNameStr = 'metal_%s_ft.txt' % pushTestNum
    text_file = open(txtFileNameStr, "w")

    for topic, msg, t in bag.read_messages(topics=['/netft_data']):
        # print msg
        text_file.write('%s \n' % msg.header.stamp)
        text_file.write('%s \n' % msg.wrench.force.x)
        text_file.write('%s \n' % msg.wrench.force.y)
    text_file.close()

    bag.close()

os.chdir( savedPath )

