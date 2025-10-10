from typing import Optional

class InputService:
    """Handles user input through speech or object detection"""
    
    def get_speech_input(self, prompt: str = "") -> Optional[str]:
        # TODO: Replace with QT-Robot speech recognition API
        if prompt:
            print(f"[ROBOT]: {prompt}")
        user_input = input("[USER SPEECH]: ")
        return user_input if user_input else None
    
    def detect_object(self) -> Optional[str]:
        # TODO: Replace with QT-Robot object detection API
        return None