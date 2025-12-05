#!/usr/bin/env python3
import sys
import rospy
from std_msgs.msg import String
from qt_robot_interface.srv import *

if __name__ == '__main__':
    rospy.init_node('my_tutorial_node')
    rospy.loginfo("my_tutorial_node started!")

    # define a ros service
    speechConfig = rospy.ServiceProxy('/qt_robot/speech/config', speech_config)

    # define a ros service
    speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
    
    # block/wait for ros service
    rospy.wait_for_service('/qt_robot/speech/say') 
    
    # block/wait for ros service
    rospy.wait_for_service('/qt_robot/speech/config') 
   
    try:
        status = speechConfig("en-US",0,0)
        if status:
            speechSay("Hello, I am speaking English")
            status = False
        rospy.sleep(1)
        #status = speechConfig("fr-FR",0,0)
        if status:
            speechSay("Bonjour, Je parle français")
        

    except KeyboardInterrupt:
        pass

    rospy.loginfo("finsihed!")
