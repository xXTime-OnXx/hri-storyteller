from core.state.state import State
from core.state_handler import StateHandler

class IntroHandler(StateHandler):
    """Handles topic selection from user"""
    
    def handle(self) -> State:
        self.robot.speak("Hi, I am Telly an interactive Storyteller")

        

        return State.TOPIC_SELECTION # Move on to the next state