from core.state.state import State
from core.state_handler import StateHandler

class IdleHandler(StateHandler):
    """Handles topic selection from user"""
    
    def handle(self) -> State:
        self.robot.speak("IDLE: Waiting for next child to approach me")
        
        # TODO: here comes face detection and VAD to detect the start of a new conversation
        
        if detected:
            return State.INTRO # Move on to the next state

        rospy.sleep(5.0) # wait for detection to run again
        return State.IDLE

        