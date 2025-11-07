from core.state.state import State
from core.state_handler import StateHandler

from service.face_tracker_service import FaceTrackerService

class IdleHandler(StateHandler):
    """Handles topic selection from user"""
    def handle(self) -> State:
        self.face_tracker.start_face_tracking()

        self.robot.speak("IDLE: Waiting for next child to approach me")
        
        detected = self.face_tracker.get_emotion()
        
        if detected:
            self.face_tracker.stop_face_tracking()
            return State.INTRO # Move on to the next state

        rospy.sleep(5.0) # wait for detection to run again
        return State.IDLE

        