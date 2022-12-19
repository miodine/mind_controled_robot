#!/usr/bin/env python3

# ROS imports
import rospy
from geometry_msgs.msg import Twist
from mcr_messages.msg import mcr_predictions

# node imports
import numpy as np


MOVE_BINDINGS = {'FRONT':(-1,0),'ROT_LEFT':(0,1),'ROT_RIGHT':(0,-1),'BACK':(1,0), 'STOP': (0,0)}
MOVE_BINDINGS_LIST = [(-1,0),(0,1),(0,-1),(1,0),(0,0)]
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

    def control_routine(self):
        # unpack control rates - for shorter notation
        control_speed = self.control_speed
        control_turn = self.control_turn
        target_speed = self.target_speed
        target_turn = self.target_turn

        if target_speed > control_speed:
            control_speed = min( target_speed, control_speed + 0.02 )
        elif target_speed < control_speed:
            control_speed = max( target_speed, control_speed - 0.02 )
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
        max_probability = max(self.current_predictions)
        i = self.current_predictions.index(max_probability)
        self.target_speed = BASE_SPEED_RATE*MOVE_BINDINGS_LIST[i][0]
        self.target_turn = BASE_TURN_RATE*MOVE_BINDINGS_LIST[i][1]


    def push_control_command(self):
        twist = Twist()
        twist.linear.x = self.control_speed; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = self.control_turn
        return twist

    def control_command(self):
        self.control_routine()
        return self.push_control_command()

def robot_controls_node():
    robot_controls = RobotControls()


    rospy.init_node("mcr_robot_controls", anonymous= False)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5) 
    sub = rospy.Subscriber("/mcr_mlclass_predictions", mcr_predictions, robot_controls.get_predictions_callback)
    
    rate = rospy.Rate(0.5)


    while not rospy.is_shutdown():
        try:
            ctl_command = robot_controls.control_command()
            pub.publish(ctl_command)
        except:
            pass
        rate.sleep()



if __name__ == "__main__":
    try:
        robot_controls_node()
    except rospy.ROSException:
        pass