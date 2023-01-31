#!/usr/bin/env python3

# ROS imports
import rospy
from std_msgs.msg import String
from mcr_messages.msg import mcr_intention

# keyboard handling imports
import sys, select, termios, tty


# Semantics

# 0 - forward
# 1 - backward
# 2 - rotate left
# 3 - rotate right
# 4 - stop 

INTENTIONS = {'w':0, 's':1, 'a': 2, 'd':3, 'x':4}
SEMANTICS = ['FORWARD', 'BACKWARD', 'ROT_LEFT', 'ROT_RIGHT', 'STOP', 'NONE']

# Helper functions
SETTINGS = termios.tcgetattr(sys.stdin)
    
def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, SETTINGS)
    return key



def bci_intention_controller_node():
    # Node initialization 
    pub = rospy.Publisher("/mcr_bci_intention", mcr_intention, queue_size=1)
    rospy.init_node("mcr_bci_intention_controller", anonymous= False)
    rospy.loginfo("Intention controller running. Press 'i' key to exit. ")
 

    # Routine init
    msg = mcr_intention()


    try:
        msg.meta = 'current intention'
        msg.intention = 4 # STOP
        msg.semantic = SEMANTICS[5]
        pub.publish(msg)
    except Exception as ex:
        rospy.loginfo(ex)
        rospy.loginfo('Shutting down now...')
        return


    while not rospy.is_shutdown():
        try: 
            key = get_key()
            if key in INTENTIONS.keys():
                msg.meta = 'current intention'
                msg.semantic = SEMANTICS[INTENTIONS[key]]
                msg.intention = INTENTIONS[key]
                rospy.loginfo("Intention for {} motion. ({})".format(msg.semantic, key))
                
                pub.publish(msg)

            if key == 'i':
                msg.meta = SEMANTICS[5]
                msg.intention = 4 # STOP
                pub.publish(msg)
                break

        except Exception as ex:
            rospy.loginfo(ex)
        finally:
            pass


if __name__ == "__main__":
    try:
        bci_intention_controller_node()
    except rospy.ROSException:
        pass