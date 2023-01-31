#!/usr/bin/env python3

# ROS imports
import rospy
from std_msgs.msg import String
from mcr_messages.msg import mcr_datastream, mcr_intention

# Data feed import 
from mcr_bci_data_loader import Brain, LOGINFO

def bci_emulator_node():
    brain = Brain()


    pub = rospy.Publisher("/mcr_bci_data_feed", mcr_datastream, queue_size=1)
    rospy.init_node("mcr_bci_emulator", anonymous= False)
    rate = rospy.Rate(1)
    rospy.loginfo(LOGINFO)

    sub = rospy.Subscriber("/mcr_bci_intention", mcr_intention, brain.get_intention_callback)



    while not rospy.is_shutdown():
        #rospy.loginfo("Length of the stream stack: {}".format(len(brain._stream_stack)))
                
        data_feed = brain.feed_data(stream_length = 1)

        msg = mcr_datastream()
        msg.meta = "...Brain signal feed..."
        msg.electrode_1 = data_feed['data'][0]
        msg.electrode_2 = data_feed['data'][1]
        msg.electrode_3 = data_feed['data'][2]
        msg.electrode_4 = data_feed['data'][3]

        msg.signal_purity = data_feed['purity']
        msg.signal_block_lenght = data_feed['block_length']
        msg.intention_interpretation = data_feed['intention']

        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        bci_emulator_node()
    except rospy.ROSException:
        pass