from core.state.state import State
from core.state_handler import StateHandler


class TopicSelectionHandler(StateHandler):
    """Handles topic selection from user"""
    
    def handle(self) -> State:
        self.robot.speak_audio(State.TOPIC_SELECTION.value)
        
        topic = self.input_service.get_speech_input()
        
        # TODO: Fallback to object detection if no speech
        # if not topic:
        #    self.robot.speak("Let me see what you're showing me...")
        #    topic = self.input_service.detect_object()
        
        if topic:
            self.context.topic = topic
            self.robot.parallel_output(
                f"Great! Let me tell you a story about {topic}",
                emotion="excited"
            )
            #return State.IDLE
            return State.STORYTELLING
        else:
            self.robot.speak("I didn't catch that. Let's try again.")
            return State.TOPIC_SELECTION  # Stay in same state