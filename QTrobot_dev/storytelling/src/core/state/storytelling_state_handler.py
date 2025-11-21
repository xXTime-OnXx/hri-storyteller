import random
from core.state.state import State
from core.state_handler import StateHandler

from core.state.prompt import create_storytelling_prompt, create_continuation_prompt

class StorytellingHandler(StateHandler):
    """Handles main storytelling loop"""

    def handle(self) -> State:
        # Generate story chunk
        if not self.context.story_history:
            chunk = self.llm.request(create_storytelling_prompt(self.context.topic))
        else:
            if self.story_count >= 6:
                chunk = self.llm.request(create_end_prompt(self.context.get_full_story()))
            else:
                chunk = self.llm.request(create_continuation_prompt(
                    self.context.get_full_story(),
                    self.context.user_last_choice
                ))
        
        self.context.add_chunk(chunk)
        
        # Tell the story chunk
        for part in chunk['story_chunks']:
            self.face_tracker.start_face_tracking()
            self.robot.parallel_output(part['text'], emotion=part['emotion'], gesture=part['gesture'])
            self.face_tracker.stop_face_tracking()

        if self.story_count >= 6:
            return state.OUTRO

        # tell next story options
        options = [choice['text'] for choice in chunk['choices']]
        options_text = "What should happen next? " + ", ".join(
            [f"{i+1}: {opt}" for i, opt in enumerate(options)]
        )
        self.robot.speak(options_text)
        
        user_choice = self.input_service.get_speech_input()
        
        if not user_choice:
            self.robot.speak("Let me decide for you...")
            user_choice = random.choice(options) 
        
        self.context.user_last_choice = user_choice

        self.story_count += 1
        
        return State.STORYTELLING