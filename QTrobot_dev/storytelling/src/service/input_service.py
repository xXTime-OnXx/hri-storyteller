import rospy
from typing import Optional
from custom_interfaces.srv import MicrophoneBasedSpeechRecognition

class InputService:
    """Handles user input through speech or object detection"""
    def __init__(self):
        try:
            rospy.wait_for_service('/custom/speech/sr/microphone_recognize')
            self.stt_service = rospy.ServiceProxy('/custom/speech/sr/microphone_recognize', MicrophoneBasedSpeechRecognition)
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

            response = self.stt_service(language)

            rospy.loginfo(response)
            
            if response and response.text:
                user_input = response.text
                if user_input:
                    rospy.loginfo(f"[USER SAID]: {user_input}")
            else:
                rospy.loginfo("[ROBOT]: I didn't hear anything.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call 'recognize' service: {e}")
        return user_input if user_input else None
    
    def detect_object(self) -> Optional[str]:
        # TODO: Replace with QT-Robot object detection API
        return None