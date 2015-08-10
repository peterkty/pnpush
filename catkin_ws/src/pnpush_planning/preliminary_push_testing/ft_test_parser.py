

import rosbag
import os

savedPath = os.getcwd()

bagFileNameStr = 'ft_test.bag'
bag = rosbag.Bag(bagFileNameStr)

text_file = open('ft_test', "w")

for topic, msg, t in bag.read_messages(topics=['/netft_data']):
    # print msg
    text_file.write('%s \n' % msg.header.stamp)
    text_file.write('%s \n' % msg.wrench.force.x)
    text_file.write('%s \n' % msg.wrench.force.y)
text_file.close()

bag.close()

os.chdir( savedPath )

