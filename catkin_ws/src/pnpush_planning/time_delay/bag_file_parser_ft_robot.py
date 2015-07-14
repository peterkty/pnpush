
import rosbag

bag = rosbag.Bag('tcp_ft_delay.bag')

text_file = open("Output_ft_fttcp.txt", "w")

for topic, msg, t in bag.read_messages(topics=['/netft_data']):
    # print msg
    text_file.write('%s \n' % msg.header.stamp)
    text_file.write('%s \n' % msg.wrench.force.x)
    text_file.write('%s \n' % msg.wrench.force.y)

text_file.close()
    
text_file = open("Output_tcp_fttcp.txt", "w")

for topic, msg, t in bag.read_messages(topics=['/tip_pose']):
    # print msg
    text_file.write('%s \n' % msg.header.stamp)
    text_file.write('%s \n' % msg.pose.position.y)
text_file.close()
bag.close()
