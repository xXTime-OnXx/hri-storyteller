from core.state.state import State
from core.state_handler import StateHandler

class IntroHandler(StateHandler):
    """Handles topic selection from user"""
    
    def handle(self) -> State:
        self.robot.speak_audio(State.INTRO.value)

        return State.TOPIC_SELECTION
