import time
import rospy

from std_msgs.msg import String

from qt_robot_interface.srv import *

class RobotService:
    """Abstraction for QT-Robot API"""
    def __init__(self):
        rospy.init_node("gemini_adapter_pub")
    
        # services
        self.speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)

        # Publishers
        self.talk_pub = rospy.Publisher("/qt_robot/behavior/talkText", String, queue_size=10)

        # wait for services and publishers
        rospy.sleep(1)  # Give ROS time to register the publisher
        rospy.wait_for_service('/qt_robot/speech/say')
    
    def speak(self, text: str):
        self.speechSay(text)
        rospy.loginfo(f"Called service /qt_robot/speech/say: {text}")

    def speak_async(self, text: str):
        msg = String()
        msg.data = text
        self.talk_pub.publish(text)
        rospy.loginfo(f"Published to /qt_robot/behavior/talkText: {text}")
    
    def show_emotion(self, emotion: str):
        # TODO: Replace with QT-Robot emotion API
        print(f"[ROBOT EMOTION]: {emotion}")
    
    def gesture(self, gesture_name: str):
        # TODO: Replace with QT-Robot gesture API
        print(f"[ROBOT GESTURE]: {gesture_name}")
    
    def parallel_output(self, text: str, emotion: str = "happy", gesture: str = None):
        # TODO: check if works and think how to track if an emotion / gesture is done or not
        """Coordinate parallel outputs"""
        self.show_emotion(emotion)
        if gesture:
            self.gesture(gesture)
        self.speak(text)