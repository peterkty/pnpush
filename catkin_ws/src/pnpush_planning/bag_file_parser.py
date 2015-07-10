
import rosbag

bag = rosbag.Bag('tcp_delay.bag')

text_file = open("Output_vicon.txt", "w")

for topic, msg, t in bag.read_messages(topics=['/vicon/markers']):
    # print msg
    text_file.write('%s \n' % msg)
text_file.close()
    
text_file = open("Output_tip.txt", "w")

for topic, msg, t in bag.read_messages(topics=['/tip_pose']):
    # print msg
    text_file.write('%s \n' % msg)
text_file.close()
bag.close()

