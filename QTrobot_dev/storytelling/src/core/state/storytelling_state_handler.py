import random
from core.state.state import State
from core.state_handler import StateHandler
import concurrent.futures

from core.state.prompt import create_storytelling_prompt, create_continuation_prompt, create_end_prompt 

class StorytellingHandler(StateHandler):
    """Handles main storytelling loop"""

    def handle(self) -> State:
        # Generate story chunk
        max_count = 1
        if not self.context.topic:
            self.context.topic = "Dragons"        
        if not self.context.story_history:
            chunk = self.llm.request(create_storytelling_prompt(self.context.topic))
        else:
            if self.story_count >= max_count:
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
        
        self.face_tracker.start_face_tracking()
        
        chunks = chunk["story_chunks"]

        self.robot.play_waiting_file()

        #Generate all chunks in parallel
        for part in chunks:
            self.context.add_chunk(part['text'])
            self.robot.generate_audio_to_queue(part['text'])

        #Generate options audio
        if self.story_count < max_count:
            options = [choice['text'] for choice in chunk['choices']]
            self.context.current_choices = "\n\n ".join(options)
            options_text = [f"{i+1}: {opt}" for i, opt in enumerate(options)]
            options_text = " ".join(options_text)
            self.robot.generate_audio_to_queue("What should happen next? " + options_text)
        
        #Tell the story chunk
        for part in chunks:
            self.robot.parallel_output_from_queue(emotion=part['emotion'], gesture=part['gesture'])
            self.robot.show_emotion("QT/neutral")
            

        if self.story_count >= max_count:
            return State.OUTRO

        #Tell the options
        self.robot.parallel_output_from_queue()
        self.robot.show_emotion("QT/neutral")
        
        user_choice = self.input_service.get_speech_input()
        
        if not user_choice:
            user_choice = random.choice(options) 
            self.robot.parallel_output("Let me decide for you...")
        
        self.context.user_last_choice = user_choice

        self.story_count += 1

        self.face_tracker.stop_face_tracking()
        
        return State.STORYTELLING