from enum import Enum
from typing import List, Optional

class State(Enum):
    INTRO = "intro"
    TOPIC_SELECTION = "topic_selection"
    STORYTELLING = "storytelling"
    OUTRO = "outro"
    FINISHED = "finished"

class StoryContext:
    def __init__(self):
        self.topic: Optional[str] = None
        self.story_history: List[str] = []
        self.current_chunk: Optional[str] = None
        self.user_last_choice: str = ""
    
    def add_chunk(self, chunk: str):
        self.story_history.append(chunk)
        self.current_chunk = chunk
    
    def get_full_story(self) -> str:
        return "\n\n".join(self.story_history)