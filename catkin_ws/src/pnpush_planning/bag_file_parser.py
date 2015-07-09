
import rosbag
bag = rosbag.Bag('2015-07-08-12-36-16.bag')
text_file = open("Output.txt", "w")

for topic, msg, t in bag.read_messages(topics=['/vicon/markers','/joint_states']):
    # print msg
    text_file.write('%s \n' % msg)
bag.close()
text_file.close()
