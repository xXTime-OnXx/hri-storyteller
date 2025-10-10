from core.state import State, StoryContext
from core.state_handler import IntroHandler, TopicSelectionHandler, StorytellingHandler, OutroHandler

from service.llm_service import LLMService
from service.robot_service import RobotService
from service.input_service import InputService


class StorytellerStateMachine:
    """Main state machine - now just coordinates state handlers"""
    
    def __init__(self):
        self.state = State.INTRO
        self.context = StoryContext()
        self.llm = LLMService()
        self.robot = RobotService()
        self.input_service = InputService()
        
        # Map states to their handlers
        self.handlers = {
            State.INTRO: IntroHandler(self.context, self.llm, self.robot, self.input_service),
            State.TOPIC_SELECTION: TopicSelectionHandler(self.context, self.llm, self.robot, self.input_service),
            State.STORYTELLING: StorytellingHandler(self.context, self.llm, self.robot, self.input_service),
            State.OUTRO: OutroHandler(self.context, self.llm, self.robot, self.input_service),
        }
    
    def run(self):
        """Main execution loop - delegates to handlers"""
        while self.state != State.FINISHED:
            handler = self.handlers.get(self.state)
            if handler:
                self.state = handler.handle()
            else:
                print(f"Unknown state: {self.state}")
                break