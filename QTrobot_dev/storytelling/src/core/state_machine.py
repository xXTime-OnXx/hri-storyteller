from core.state.state import State, StoryContext
from core.state_handler import StateHandler
from core.state.intro_state_handler import IntroHandler
from core.state.topic_state_handler import TopicSelectionHandler
from core.state.storytelling_state_handler import StorytellingHandler
from core.state.outro_state_handler import OutroHandler
from core.state.idle_state_handler import IdleHandler

from service.llm_service import LLMService
from service.robot_service import RobotService
from service.input_service import InputService
from service.face_tracker_service import FaceTrackerService

class StorytellerStateMachine:
    """Main state machine - now just coordinates state handlers"""
    
    def __init__(self):
        self.state = State.INTRO
        self.context = StoryContext()
        self.llm = LLMService()
        self.robot = RobotService()
        self.input_service = InputService()
        self.face_tracker = FaceTrackerService()
        
        # Map states to their handlers
        self.handlers = {
            State.INTRO: IntroHandler(self.context, self.llm, self.robot, self.input_service, self.face_tracker),
            State.TOPIC_SELECTION: TopicSelectionHandler(self.context, self.llm, self.robot, self.input_service, self.face_tracker),
            State.STORYTELLING: StorytellingHandler(self.context, self.llm, self.robot, self.input_service, self.face_tracker),
            State.OUTRO: OutroHandler(self.context, self.llm, self.robot, self.input_service, self.face_tracker),
            State.IDLE: IdleHandler(self.context, self.llm, self.robot, self.input_service, self.face_tracker),
        }
    
    def run(self):
        """Main execution loop - delegates to handlers"""
        while True: # TODO: add safety feature that can stop the process instead of inifite loop
            handler = self.handlers.get(self.state)
            if handler:
                self.state = handler.handle()
            else:
                print(f"Unknown state: {self.state}")
                break