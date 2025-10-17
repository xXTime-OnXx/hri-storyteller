from core.state import State, StoryContext
from service.llm_service import LLMService
from service.robot_service import RobotService
from service.input_service import InputService

class StateHandler:
    def __init__(self, context: StoryContext, llm: LLMService, 
                 robot: RobotService, input_service: InputService):
        self.context = context
        self.llm = llm
        self.robot = robot
        self.input_service = input_service
    
    def handle(self) -> State:
        raise NotImplementedError

# IntroHandler, TopicSelectionHandler, StorytellingHandler, OutroHandler
# Also separate into multiple files

class IntroHandler(StateHandler):
    """Handles topic selection from user"""
    
    def handle(self) -> State:
        self.robot.speak("Hi, I am Telly an interactive Storyteller")
        
        return State.TOPIC_SELECTION # Move on to the next state

class TopicSelectionHandler(StateHandler):
    """Handles topic selection from user"""
    
    def handle(self) -> State:
        self.robot.speak("What would you like the story to be about?")
        
        # Try speech first
        topic = self.input_service.get_speech_input()
        
        # Fallback to object detection if no speech
        if not topic:
            self.robot.speak("Let me see what you're showing me...")
            topic = self.input_service.detect_object()
        
        if topic:
            self.context.topic = topic
            self.robot.parallel_output(
                f"Great! Let me tell you a story about {topic}",
                emotion="excited"
            )
            return State.STORYTELLING
        else:
            self.robot.speak("I didn't catch that. Let's try again.")
            return State.TOPIC_SELECTION  # Stay in same state


class StorytellingHandler(StateHandler):
    """Handles main storytelling loop"""
    
    def handle(self) -> State:
        # Generate story chunk
        if not self.context.story_history:
            chunk = self.llm.generate_story_start(self.context.topic)
        else:
            chunk = self.llm.generate_story_continuation(
                self.context.get_full_story(),
                self.context.user_last_choice
            )
        
        self.context.add_chunk(chunk)
        
        # Tell the story chunk
        self.robot.parallel_output(chunk, emotion="storytelling", gesture="narrate")
        
        # Check if story should end
        if self.llm.should_story_end(self.context.get_full_story()):
            return State.OUTRO
        
        # Generate options and get user choice
        options = self.llm.generate_options(self.context.get_full_story())
        
        options_text = "What should happen next? " + ", ".join(
            [f"{i+1}: {opt}" for i, opt in enumerate(options)]
        )
        self.robot.speak(options_text)
        
        user_choice = self.input_service.get_speech_input()
        
        if not user_choice:
            self.robot.speak("Let me decide for you...")
            user_choice = options[0]
        
        self.context.user_last_choice = user_choice
        
        return State.STORYTELLING  # Continue loop


class OutroHandler(StateHandler):
    """Handles conclusion of storytelling"""
    
    def handle(self) -> State:
        outro_text = "And that's the end of our story! Thank you for this wonderful adventure together!"
        self.robot.parallel_output(outro_text, emotion="happy", gesture="bow")
        return State.FINISHED