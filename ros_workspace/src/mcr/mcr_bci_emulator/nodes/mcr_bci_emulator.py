#!/usr/bin/env python3

# ROS imports
import rospy
from std_msgs.msg import String
from mcr_messages.msg import mcr_datastream

# Data feed import 
from mcr_bci_data_loader import Brain, LOGINFO

def bci_emulator_node():
    pub = rospy.Publisher("/mcr_bci_data_feed", mcr_datastream, queue_size=18)
    rospy.init_node("mcr_bci_emulator", anonymous= False)
    rate = rospy.Rate(1)
    rospy.loginfo(LOGINFO)

    brain = Brain()

    while not rospy.is_shutdown():
        #rospy.loginfo("Length of the stream stack: {}".format(len(brain._stream_stack)))
        signal_block = brain.feed_data()

        msg = mcr_datastream()
        msg.meta = "...Brain signal feed..."
        msg.electrode_1 = signal_block[0]
        msg.electrode_2 = signal_block[1]
        msg.electrode_3 = signal_block[2]
        msg.electrode_4 = signal_block[3]

        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        bci_emulator_node()
    except rospy.ROSException:
        pass