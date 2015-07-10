
import rosbag
bag = rosbag.Bag('tcp_delay.bag')
text_file = open("Output.txt", "w")

for topic, msg, t in bag.read_messages(topics=['/vicon/markers','/tip_pose']):
    # print msg
    text_file.write('%s \n' % msg)
bag.close()
text_file.close()
