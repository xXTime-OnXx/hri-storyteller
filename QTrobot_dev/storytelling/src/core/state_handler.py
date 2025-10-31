from core.state.state import State, StoryContext
from service.llm_service import LLMService
from service.robot_service import RobotService
from service.input_service import InputService
from service.face_tracker_service import FaceTrackerService

class StateHandler:
    def __init__(self, context: StoryContext, llm: LLMService, 
                 robot: RobotService, input_service: InputService,
                 face_tracker: FaceTrackerService):
        self.context = context
        self.llm = llm
        self.robot = robot
        self.input_service = input_service
        self.face_tracker = face_tracker
    
    def handle(self) -> State:
        raise NotImplementedError
