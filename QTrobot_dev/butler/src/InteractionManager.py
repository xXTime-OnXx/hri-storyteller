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
        SMALL_TALK = auto()
        GOODBYE = auto()
        INTRODUCTION = auto()

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
                
                elif step == self.Type.SMALL_TALK:
                    self._script_small_talk(person)

                elif step == self.Type.GOODBYE:
                    self._script_farewell(person)

                elif step == self.Type.INTRODUCTION:
                    self._script_introduce_unknown(person)

                    
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
        intro = "Good day, Madam." if person.gender == "female" else "Good day, Sir."
        self._say(intro)
        self._say("I do not believe we have met.")
        self._say("May I inquire as to your name?")

        # Listen for name (blocking)
        resp = self._listen(options=[], timeout=10)
        name_raw = self._clean_name(resp.transcript if resp else "")

        if not name_raw:
            self._say("Forgive me, I did not catch that.", "QT/Dance-1-2")
            return

        # Optional verification
        if not self._verify_name(name_raw):
            self._say("My apologies. I will try to learn your name another time.", "QT/Waving_rightArm")
            return

        # Save name
        person.name = name_raw
        self.brain.register_new_person(person)

        self._say(f"A pleasure to meet you, {name_raw}.")

    def _script_greet_known(self, person):
        name = person.name or ("Sir" if person.gender == "Man" else "Ma'am")
        self._say(f"Welcome back, {name}.")

    def _script_encourage(self, person):
        line = random.choice(self.encouragements)
        self._say(line, "QT/.shoot.xml")

    def _script_compliment(self, person):
        if person.emotion == "happy":
            line = random.choice(self.compliments_smile)
            self._say(line, "QT/happy")
        else:
            line = random.choice(self.compliments_general)
            self._say(line, "QT/Fly")
    
    def _script_small_talk(self, person):
        self._say("May I ask you something?")
        resp = self._listen(options=["yes", "no"], timeout=5)

        if not resp or "no" in resp.transcript.lower():
            self._say("Very well. If you ever wish to chat, I shall be here.")
            return

        # Step 2: Ask a general question
        self._say("How is your day going so far?")
        answer = self._listen(options=[], timeout=10)

        if not answer or not answer.transcript:
            self._say("I see. Perhaps it is a quiet day.")
            return

        text = answer.transcript.lower()

        # Step 3: Simple response classification
        if any(w in text for w in ["good", "great", "fine", "nice", "amazing", "okay"]):
            self._say("I am delighted to hear that.")
        elif any(w in text for w in ["bad", "tired", "sad", "not good", "awful"]):
            self._say("I am sorry to hear that. I hope things improve soon.")
        else:
            self._say("That sounds interesting. Thank you for sharing.")
    
    def _script_farewell(self, person):
        farewell_lines = [
            "May your steps today lead you toward something meaningful.",
            "Wisdom grows in quiet moments. I hope you find a few today.",
            "Take good care of yourself. Small acts of kindness matter most.",
            "May calm thoughts guide your choices and brighten your path.",
            "Remember, even a brief moment of stillness can renew the mind.",
            "Strength often comes from patience, not haste. Walk gently.",
        ]

        line = random.choice(farewell_lines)

        # Optional: add a gesture (just an example — choose one your robot has)
        self._say(line + person.name if person.is_known() else "", gesture_name="QT/wave")
    
    def _script_introduce_unknown(self, person):
        self._say(f'Allow me to introduce myself {person.name}. I am QT Bot, your household butler, here to assist you with courtesy and reliability.')
        self._say(f'It is a pleasure to make your acquaintance, {person.name}. I look forward to assisting you whenever you may need me.', 'QT/happy')

        

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

