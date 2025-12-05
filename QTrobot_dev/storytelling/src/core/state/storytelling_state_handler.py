import random
from core.state.state import State
from core.state_handler import StateHandler

from core.state.prompt import create_storytelling_prompt, create_continuation_prompt, create_end_prompt

class StorytellingHandler(StateHandler):
    """Handles main storytelling loop"""

    def handle(self) -> State:
        # Generate story chunk
        if not self.context.topic:
            self.context.topic = "Dragons"        
        if not self.context.story_history:
            chunk = self.llm.request(create_storytelling_prompt(self.context.topic))
        else:
            if self.story_count >= 5:
                chunk = self.llm.request(create_end_prompt(
                    self.context.get_full_story(), 
                    self.context.current_choices,
                    self.context.user_last_choice
                ))
            else:
                chunk = self.llm.request(create_continuation_prompt(
                    self.context.get_full_story(),
                    self.context.current_choices,
                    self.context.user_last_choice
                ))
        
        self.robot.play_waiting_file()
        
        # Tell the story chunk
        for part in chunk['story_chunks']:
            self.context.add_chunk(part['text'])
            self.face_tracker.start_face_tracking()
            self.robot.parallel_output(part['text'], emotion=part['emotion'], gesture=part['gesture'])
            self.face_tracker.stop_face_tracking()
            

        if self.story_count >= 5:
            return State.OUTRO

        # tell next story options
        options = [choice['text'] for choice in chunk['choices']]
        print(options)
        self.context.current_choices = "\n\n ".join(options)

        options_text = [f"{i+1}: {opt}" for i, opt in enumerate(options)]
        
        print(options_text)
        self.robot.parallel_output("What should happen next? " + options_text)
        
        user_choice = self.input_service.get_speech_input()
        
        if not user_choice:
            self.robot.parallel_output("Let me decide for you...")
            user_choice = random.choice(options) 
        
        self.context.user_last_choice = user_choice

        self.story_count += 1
        
        return State.STORYTELLING