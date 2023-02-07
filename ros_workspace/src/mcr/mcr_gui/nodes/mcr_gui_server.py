#!/usr/bin/env python

import rospy
from mcr_messages.msg import *


class DataServer:
    def __init__(self):
        # datastream
        self.electrode_1 = None
        self.electrode_2 = None
        self.electrode_3 = None
        self.electrode_4 = None
        self.signal_purity = None
        self.signal_block_lenght = None
        self.intention_interpretation = None 
        
        # control monit
        self.predicted_intention = None
        self.confidence = None 
        self.all_predictions = None 


        # 

    



    def _initialize_subscribers(self):
        rospy.Subscriber('/mcr_mlclass_predictions', mcr_predictions)
        rospy.Subscriber('/mcr_control_monit', mcr_control_monit)
        rospy.Subscriber("/mcr_bci_data_feed", mcr_datastream)
        rospy.Subscriber("/mcr_bci_intention", mcr_intention)