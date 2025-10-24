import rospy
from typing import Optional
from qt_vosk_app.srv import *

class InputService:
    """Handles user input through speech or object detection"""
    def __init__(self):
        try:
            self.stt_service = rospy.ServiceProxy('/qt_robot/speech/recognize', speech_recognize)
            rospy.wait_for_service('/qt_robot/speech/recognize')
            rospy.loginfo("Speech services connected successfully.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to create service proxies: {e}")
            return
    
    def get_speech_input(self, prompt: str = "") -> Optional[str]:
        if prompt:
            print(f"[ROBOT]: {prompt}")

        user_input = None
        try:
            language = "en-US" 
            timeout = 10

            response = self.stt_service(language, ["green", "blue"], timeout)
            
            if response and response.transcript:
                user_input = response.transcript.strip()
                if user_input:
                    rospy.loginfo(f"[USER SAID]: {user_input}")
                else:
                    rospy.loginfo("[ROBOT]: I heard something, but it was empty.")
            else:
                rospy.loginfo("[ROBOT]: I didn't hear anything.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call 'recognize' service: {e}")
        return user_input if user_input else None
    
    def detect_object(self) -> Optional[str]:
        # TODO: Replace with QT-Robot object detection API
        return None