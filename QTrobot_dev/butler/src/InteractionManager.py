#!/usr/bin/env python3
import rospy
import threading
import random
from enum import Enum, auto

from qt_gesture_controller.srv import gesture_play
from TaskSynchronizer import TaskSynchronizer
from qt_robot_interface.srv import *
from qt_vosk_app.srv import *


class InteractionManager:

    class Type(Enum):
        GREET_UNKNOWN = auto()
        GREET_KNOWN = auto()
        ENCOURAGE = auto()
        COMPLIMENT = auto()

    def __init__(self, brain):
        self.brain = brain

        # Connect speech services
        rospy.loginfo("[InteractionManager] Waiting for speech services...")
        rospy.wait_for_service('/qt_robot/speech/say')
        rospy.wait_for_service('/qt_robot/speech/recognize')

        rospy.wait_for_service('/qt_robot/behavior/talkText')
        rospy.wait_for_service('/qt_robot/gesture/play')

        self.talk_srv = rospy.ServiceProxy('/qt_robot/behavior/talkText', behavior_talk_text)
        self.gesture_srv = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)

        self.ts = TaskSynchronizer()

        self.say_srv = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
        self.rec_srv = rospy.ServiceProxy('/qt_robot/speech/recognize', speech_recognize)

        rospy.loginfo("[InteractionManager] Speech services available.")

        # Predefined sentence pools
        self.encouragements = [
            "Do not be discouraged. Better days lie ahead.",
            "Please remember that challenges pass with time.",
            "I am here if you need assistance or a kind word."
        ]

        self.compliments_general = [
            "You look wonderful today.",
            "Your presence brightens the room.",
            "Your style is remarkably tasteful."
        ]

        self.compliments_smile = [
            "Your smile is truly charming.",
            "A delightful smile like yours brightens the day.",
            "Your smile carries a warm and uplifting spirit."
        ]

    # ================================================================
    # PUBLIC: ENTRY POINT CALLED BY BUTLERBRAIN
    # ================================================================

    def start_interaction(self, person, interaction_steps):
        """
        Starts interaction in its own thread.
        ButlerBrain switches to INTERACTING state and will stay there
        until finish_interaction(...) is called.
        """
        t = threading.Thread(
            target=self._interaction_thread,
            args=(person, interaction_steps)
        )
        t.daemon = True
        t.start()

    # ================================================================
    # INTERNAL: THIS RUNS IN A SEPARATE THREAD
    # ================================================================

    def _interaction_thread(self, person, interaction_steps):
        # Ensure input is a list, even if a single item is passed
        if not isinstance(interaction_steps, list):
            interaction_steps = [interaction_steps]
            
        rospy.loginfo(f"[InteractionManager] Running sequence {interaction_steps} with {person.id}")

        try:
            for step in interaction_steps:
                
                # Check if the robot should stop (e.g. ROS shutdown or emergency flag)
                if rospy.is_shutdown(): 
                    break

                rospy.loginfo(f"[InteractionManager] Executing step: {step}")

                if step == self.Type.GREET_UNKNOWN:
                    self._script_greet_unknown(person)

                elif step == self.Type.GREET_KNOWN:
                    self._script_greet_known(person)

                elif step == self.Type.ENCOURAGE:
                    self._script_encourage(person)

                elif step == self.Type.COMPLIMENT:
                    self._script_compliment(person)
                    
                # Add a small natural pause between steps so it doesn't sound robotic
                rospy.sleep(0.5)

        except Exception as e:
            rospy.logerr(f"[InteractionManager] Error during script sequence: {e}")

        finally:
            # Notify the brain that the ENTIRE sequence has ended
            self.brain.finish_interaction(person)

    # ================================================================
    # INTERACTION SCRIPTS (LINEAR LOGIC)
    # ================================================================

    def _script_greet_unknown(self, person):
        intro = "Good day, Madam." if person.gender == "Woman" else "Good day, Sir."
        self._say(intro)
        self._say("I do not believe we have met.")
        self._say("May I inquire as to your name?")

        # Listen for name (blocking)
        resp = self._listen(options=[], timeout=10)
        name_raw = self._clean_name(resp.transcript if resp else "")

        if not name_raw:
            self._say("Forgive me, I did not catch that.", "bye-bye")
            return

        # Optional verification
        if not self._verify_name(name_raw):
            self._say("My apologies. I will try to learn your name another time.", "bye-bye")
            return

        # Save name
        person.name = name_raw
        self.brain.register_new_person(person.id, name_raw)

        self._say(f"A pleasure to meet you, {name_raw}.")

    def _script_greet_known(self, person):
        name = person.name or ("Sir" if person.gender == "Man" else "Ma'am")
        self._say(f"Welcome back, {name}.")
        if(person.name == 'Marco'):
            self._say("i didnt know youre into men.", "monkey")

    def _script_encourage(self, person):
        line = random.choice(self.encouragements)
        self._say(line, ".shoot.xml")

    def _script_compliment(self, person):
        if person.emotion == "happy":
            line = random.choice(self.compliments_smile)
            self._say(line, "happy")
        else:
            line = random.choice(self.compliments_general)
            self._say(line, "Fly")
        

    # ================================================================
    # LOW-LEVEL SPEAK & LISTEN FUNCTIONS
    # ================================================================

    def _say(self, text, gesture_name=None):
        tasks = [(0, lambda: self.talk_srv(text))]
        if gesture_name:
            tasks.append((0, lambda: self.gesture_srv(gesture_name, 0)))
        self.ts.sync(tasks)


    def _listen(self, options, timeout, language="en_US"):
        """
        options = []   → free speech
        options = ["yes", "no"] → forced yes/no recognition
        """
        rospy.loginfo("[InteractionManager] LISTENING...")
        try:
            return self.rec_srv(language, options, timeout)
        except Exception as e:
            rospy.logerr(f"[InteractionManager] Listen error: {e}")
            return None

    def _verify_name(self, name):
        self._say(f"I heard {name}. Is that correct?")
        resp = self._listen(options=["yes", "no"], timeout=5)

        if not resp or not resp.transcript:
            return False
        return "yes" in resp.transcript.lower()

    def _clean_name(self, text):
        if not text:
            return None
        return text.split()[-1].strip(",.!?").capitalize()
