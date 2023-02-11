#!/usr/bin/env python3

# ROS imports
from geometry_msgs.msg import Twist
from mcr_messages.msg import mcr_control_monit

# module imports
import numpy as np
import os


# PTF = path to file
PTF = str(os.path.dirname(__file__))

# load run info
__log_txt_file_handle = open(PTF + "/presentation_layer/init_msg.txt","r")
LOGINFO = __log_txt_file_handle.read()
__log_txt_file_handle.close()



# Robot control constants
MOVE_BINDINGS = {'FORWARD':(-1,0),'BACKWARD':(1,0), 'ROT_LEFT':(0,-1),'ROT_RIGHT':(0,1),'STOP': (0,0)}
MOVE_BINDINGS_LIST = [(-1,0),(1,0),(0,-1),(0,1),(0,0)]
BASE_SPEED_RATE = .6
BASE_TURN_RATE = 1

class RobotControls:
    def __init__(self):
        self.control_speed = 0
        self.control_turn = 0
        self.current_predictions = [0,0,0,0,0]
        self.predicted_mov_dir = 0
        
        self.target_speed = 0
        self.target_turn = 0
        self.max_probability = 0


    def control_routine(self):
        # unpack control rates - for shorter notation
        control_speed = self.control_speed
        control_turn = self.control_turn
        target_speed = self.target_speed
        target_turn = self.target_turn

        if target_speed > control_speed:
            control_speed = min( target_speed, control_speed + 0.5 )
        elif target_speed < control_speed:
            control_speed = max( target_speed, control_speed - 0.5 )
        else:
            control_speed = target_speed

        if target_turn > control_turn:
            control_turn = min( target_turn, control_turn + 0.1 )
        elif target_turn < control_turn:
            control_turn = max( target_turn, control_turn - 0.1 )
        else:
            control_turn = target_turn

        # re-pack control rates
        self.control_speed = control_speed
        self.control_turn = control_turn
        self.target_speed = target_speed
        self.target_turn = target_turn

    def get_predictions_callback(self,data):
        self.current_predictions = list(data.predictions)
        #TEMPORARY PATCH
        self.max_probability = max(self.current_predictions)
        i = self.current_predictions.index(self.max_probability)
        self.target_speed = BASE_SPEED_RATE*MOVE_BINDINGS_LIST[i][0]
        self.target_turn = BASE_TURN_RATE*MOVE_BINDINGS_LIST[i][1]

    def monit(self):
        msg = mcr_control_monit()

        i = self.current_predictions.index(self.max_probability)
        msg.predicted_intention = list(MOVE_BINDINGS.keys())[i]
        msg.confidence = self.max_probability
        msg.all_predictions = self.current_predictions
        return msg 

    def push_control_command(self):
        twist = Twist()
        twist.linear.x = self.control_speed; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = self.control_turn
        return twist

    def control_command(self):
        self.control_routine()
        return self.push_control_command()