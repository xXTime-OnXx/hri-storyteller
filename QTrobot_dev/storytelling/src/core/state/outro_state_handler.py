from core.state.state import State
from core.state_handler import StateHandler


class OutroHandler(StateHandler):
    """Handles conclusion of storytelling"""
    
    def handle(self) -> State:
        outro_text = "And that's the end of our story! Thank you for this wonderful adventure together!"
        self.robot.parallel_output(outro_text, emotion="happy", gesture="bow")
        return State.IDLE