#!/usr/bin/env python3
import rospy
import copy
import math
from sensor_msgs.msg import JointState



def convert_callback(msg):
    converted_joint_state = copy.deepcopy(msg)
    positions = list(converted_joint_state.position)

    for i in range(len(positions)):
         positions[i] = math.radians(positions[i])
    
    converted_joint_state.position = tuple(positions)
    pub.publish(converted_joint_state)


    
def joint_state_converter():


    rospy.Subscriber("/qt_robot/joints/state", JointState, convert_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('joint_state_converter', anonymous=True)
    pub = rospy.Publisher('joint_states_in_radians', JointState, queue_size=10)
    rospy.sleep(3)
    joint_state_converter()
