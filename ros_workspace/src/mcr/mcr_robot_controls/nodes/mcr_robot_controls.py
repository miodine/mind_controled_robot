#!/usr/bin/env python3

# ROS imports
import rospy
from geometry_msgs.msg import Twist
from mcr_messages.msg import mcr_predictions

# node imports
from mcr_control_routines import RobotControls, LOGINFO


def robot_controls_node():
    robot_controls = RobotControls()

    rospy.init_node("mcr_robot_controls", anonymous= False)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5) 
    sub = rospy.Subscriber("/mcr_mlclass_predictions", mcr_predictions, robot_controls.get_predictions_callback)    
    rate = rospy.Rate(0.5)
    rospy.loginfo(LOGINFO)



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