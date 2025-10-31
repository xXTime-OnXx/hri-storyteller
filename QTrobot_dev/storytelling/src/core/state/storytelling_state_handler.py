from core.state.state import State
from core.state_handler import StateHandler

from core.state.prompt import create_storytelling_prompt

class StorytellingHandler(StateHandler):
    """Handles main storytelling loop"""

    def handle(self) -> State:
        # TODO: overwrite topic temporarily
        # self.context.topic = "Dragon"

        # Generate story chunk
        if not self.context.story_history:
            chunk = self.llm.request(create_storytelling_prompt(self.context.topic))
        else:
            chunk = self.llm.generate_story_continuation(
                self.context.get_full_story(),
                self.context.user_last_choice
            )
        
        self.context.add_chunk(chunk)
        
        # Tell the story chunk
        for part in chunk['story_chunks']:
            self.robot.parallel_output(part['text'], emotion=part['emotion'], gesture=part['gesture'])

        # TODO: check if story should end?
        
        # tell next story options
        options = [choice['text'] for choice in chunk['choices']]
        options_text = "What should happen next? " + ", ".join(
            [f"{i+1}: {opt}" for i, opt in enumerate(options)]
        )
        self.robot.speak(options_text)
        
        user_choice = self.input_service.get_speech_input()
        
        if not user_choice:
            self.robot.speak("Let me decide for you...")
            user_choice = options[0]
        
        self.context.user_last_choice = user_choice
        
        # return State.STORYTELLING # Continue loop
        return State.OUTRO # temporarily skip this step