#!/usr/bin/env python3

# ROS imports
import rospy
from std_msgs.msg import String
from mcr_messages.msg import mcr_datastream, mcr_predictions

# node imports
import numpy as np
from mcr_model import MlClassifier, LOGINFO



def ml_classifier_node():
    mlc = MlClassifier()


    rospy.init_node("mcr_ml_classifier", anonymous= False)
    pub = rospy.Publisher("/mcr_mlclass_predictions", mcr_predictions, queue_size=10)
    sub = rospy.Subscriber("/mcr_bci_data_feed", mcr_datastream, mlc.get_signals_callback)
    rate = rospy.Rate(0.5)

    rospy.loginfo(LOGINFO)

    while not rospy.is_shutdown():
        try:
            msg = mlc.classify()
            pub.publish(msg)
        except:
            pass
        
        
        rate.sleep()



if __name__ == "__main__":
    try:
        ml_classifier_node()
    except rospy.ROSException:
        pass