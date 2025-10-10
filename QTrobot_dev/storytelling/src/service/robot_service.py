import time

class RobotService:
    """Abstraction for QT-Robot API"""
    
    def speak(self, text: str):
        # TODO: Replace with QT-Robot TTS API
        print(f"[ROBOT SPEAKS]: {text}")
        time.sleep(1)
    
    def show_emotion(self, emotion: str):
        # TODO: Replace with QT-Robot emotion API
        print(f"[ROBOT EMOTION]: {emotion}")
    
    def gesture(self, gesture_name: str):
        # TODO: Replace with QT-Robot gesture API
        print(f"[ROBOT GESTURE]: {gesture_name}")
    
    def parallel_output(self, text: str, emotion: str = "happy", gesture: str = None):
        """Coordinate parallel outputs"""
        self.show_emotion(emotion)
        if gesture:
            self.gesture(gesture)
        self.speak(text)