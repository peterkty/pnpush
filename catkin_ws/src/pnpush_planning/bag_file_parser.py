
import rosbag

bag = rosbag.Bag('tcp_delay.bag')

text_file = open("Output_vicon.txt", "w")

for topic, msg, t in bag.read_messages(topics=['/vicon/markers']):
    # print msg
    text_file.write('%s \n' % msg.header.stamp)
    text_file.write('%s \n' % msg.markers[-1].translation.x)

text_file.close()
    
text_file = open("Output_tip.txt", "w")

for topic, msg, t in bag.read_messages(topics=['/tip_pose']):
    # print msg
    text_file.write('%s \n' % msg.header.stamp)
    text_file.write('%s \n' % msg.pose.position.x)
text_file.close()
bag.close()

