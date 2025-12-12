import rospy
from FaceManager2 import FaceManager
from InteractionManager import InteractionManager
from HeadController import HeadController
from Person import Person
from std_msgs.msg import Float64MultiArray
from enum import Enum, auto
from qt_robot_interface.srv import *
from qt_gesture_controller.srv import gesture_play


class ButlerBrain:

    class State(Enum):
        IDLE = auto()
        INTERACTING = auto()

    def __init__(self):
        rospy.init_node("qt_butler")

        self.hc = HeadController()
        self.fm = FaceManager()
        self.im = InteractionManager(self)
        self.rate = rospy.Rate(10)
        self.state = self.State.IDLE

        self.current_target = None

        rospy.wait_for_service('/qt_robot/behavior/talkText')
        rospy.wait_for_service('/qt_robot/gesture/play')
        self.gesture_srv = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
        self.talk_srv = rospy.ServiceProxy('/qt_robot/behavior/talkText', behavior_talk_text)

    def run(self):
        #default pose
        #self.talk_srv("Robo Ranger")
        self.gesture_srv("QT/neutral", 0) 

        while not rospy.is_shutdown():
            active_faces = self.fm.get_active_faces()
            print(active_faces)
            
            if self.current_target is not None:
                # Update current_target's coordinates from the live feed
                #self.update_target_coordinates(active_faces)
                # Physically move head to look at them
                self.hc.track_face(self.current_target.region)

            # --- DECISION LOGIC (The Brain) ---
            
            if self.state == self.State.IDLE:
                # Only look for NEW people if we are not busy
                target = self.decide_target_person(active_faces)
                
                if target is not None:
                    self.start_interaction(target)

            # If State is INTERACTING, we do nothing here. 
            # We just wait for the thread to finish and reset us to IDLE.

            self.rate.sleep()

    def decide_target_person(self, faces: dict):
        candidates = [f for f in faces if not f.timeout_active()]
        
        if not candidates:
            return None

        candidates.sort(key=lambda x: (
            x.emotion == 'sad',      # True comes first
            x.gender == 'female',    # True comes first
            not x.is_known()        # Known people usually higher priority? Or greet guests first? You decide.
        ), reverse=True)
        
        return candidates[0]

    def look_at_face(self, person: Person):
        x = person.region['x']
        y = person.region['y']

        href = Float64MultiArray()
        href.data = [head_yaw_ref, 0]
        head_pub.publish(href)
        

    def start_interaction(self, person):
        self.state = self.State.INTERACTING
        self.current_target = person

        sequence = []

        if person.is_known():
            sequence.append(InteractionManager.Type.GREET_KNOWN)

            if person.emotion == 'sad':
                sequence.append(InteractionManager.Type.ENCOURAGE)
            else:
                sequence.append(InteractionManager.Type.COMPLIMENT)

            sequence.append(InteractionManager.Type.SMALL_TALK)
            sequence.append(InteractionManager.Type.GOODBYE)
        else:
            sequence.append(InteractionManager.Type.GREET_UNKNOWN)
            sequence.append(InteractionManager.Type.INTRODUCTION)

        self.im.start_interaction(person, sequence)


    def finish_interaction(self, person):
        person.last_ineraction = rospy.get_time()
        self.current_target = None
        self.state = self.State.IDLE
        self.gesture_srv("QT/neutral", 0) 
    
    def register_new_person(self, person):
        self.fm.register_new_face(person)

if __name__ == "__main__":
    br = ButlerBrain()
    br.run()