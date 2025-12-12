import time
import random
import rospy

from std_msgs.msg import String

from qt_robot_interface.srv import *

from TTS.api import TTS
import subprocess
import asyncio

from queue import Queue

class RobotService:
    """Abstraction for QT-Robot API"""
    def __init__(self):
        rospy.init_node("gemini_adapter_pub")
    
        # services
        self.speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
        self.audioPlay = rospy.ServiceProxy('/qt_robot/audio/play', audio_play)

        # Publishers
        self.talk_pub = rospy.Publisher("/qt_robot/behavior/talkText", String, queue_size=10)
        self.emotion_publisher = rospy.Publisher('/qt_robot/emotion/show', String, queue_size=10)
        self.gesture_publisher = rospy.Publisher('/qt_robot/gesture/play', String, queue_size=10)

        # wait for services and publishers
        # rospy.sleep(1)  # Give ROS time to register the publisher
        # rospy.wait_for_service('/qt_robot/speech/say')
        # rospy.wait_for_service('/qt_robot/audio/play')

        rospy.sleep(1)
        rospy.wait_for_service('/qt_robot/audio/play')
        rospy.wait_for_service('/qt_robot/audio/play_local_file')

        rospy.loginfo("Loading Coqui TTS model...")
        self.tts = TTS(model_name="tts_models/en/vctk/vits", progress_bar=False, gpu=False)
        self.speaker_id = "p230"  # male voice
        rospy.loginfo("TTS model loaded.")

        self.audio_queue = Queue()
    
    def generate_file(self, text: str, speed: float = 0.9, volume_db: int = 5):
        wav_file = f"/root/catkin_ws/src/audios/{time.time()}.wav"

        # Load Coqui TTS model
        tts = TTS(model_name="tts_models/en/vctk/vits", progress_bar=False, gpu=False)
        model_loaded = time.time()
        
        # Choose a male speaker
        speaker_id = "p230"

        # Generate WAV at 16kHz directly to the output file
        tts.tts_to_file(
            text=text,
            speaker=speaker_id,
            file_path=wav_file,
            audio_config={"sample_rate": 16000}
        )

        speed = 0.85  # 1.0 = normal, >1.0 = faster, <1.0 = slower
        tmp_file = wav_file.replace(".wav", ".tmp.wav")  # proper WAV extension

        subprocess.run([
            "ffmpeg", "-y", "-i", wav_file,
            "-filter:a", f"volume=5dB,atempo={speed}",
            tmp_file
        ], check=True)

        # Replace original file with adjusted version
        subprocess.run(["mv", tmp_file, wav_file], check=True)

        return wav_file

    def play_file(self, wav_file, delete = False):
        # Play the final adjusted file
        subprocess.run([
            "rosservice", "call",
            "/qt_robot/audio/play_local_file",
            wav_file
        ])
        #TODO: Delete File if delete is true

    def play_waiting_file(self):
        
        waiting_file_list = ["waiting_1.wav", "waiting_2.wav", "waiting_3.wav", "waiting_4.wav", "waiting_5.wav", "waiting_6.wav", "waiting_7.wav", "waiting_8.wav", "waiting_9.wav", "waiting_10.wav"]
        
        waiting_file = "/root/catkin_ws/src/audios/" + random.choice(waiting_file_list)

        subprocess.run([
            "rosservice", "call",
            "/qt_robot/audio/play_local_file",
            waiting_file
        ])

    def speak_audio(self, state: str):
        self.audioPlay(state + '.wav', '/home/developer')

    def speak_async(self, text: str):
        msg = String()
        msg.data = text
        self.talk_pub.publish(text)
        rospy.loginfo(f"Published to /qt_robot/behavior/talkText: {text}")
    
    def show_emotion(self, emotion: str, talk: bool = False):
        """
        Show emotion on QT Robot asynchronously via ROS publisher
        
        Args:
            emotion: Emotion name (e.g., 'QT/happy', 'QT/sad', 'QT/happy_blinking')
        """
        rospy.loginfo(f"[ROBOT EMOTION]: {emotion}")
        msg = String()
        msg.data = emotion
        self.emotion_publisher.publish(msg)
        if talk:
            msg = String()
            msg.data = "QT/talking"
            self.emotion_publisher.publish(msg)
    
    def gesture(self, gesture_name: str):
        """
        Perform gesture on QT Robot asynchronously via ROS publisher
        
        Args:
            gesture_name: Gesture name (e.g., 'QT/happy', 'QT/Fly', 'QT/hands-on-belly')
        """
        rospy.loginfo(f"[ROBOT GESTURE]: {gesture_name}")
        msg = String()
        msg.data = gesture_name
        self.gesture_publisher.publish(msg)
    
    def parallel_output(self, text: str, emotion: str = "happy", gesture: str = None):
        # TODO: check if works and think how to track if an emotion / gesture is done or not
        """Coordinate parallel outputs"""
        wav_file = self.generate_file(text)

        self.show_emotion(emotion)
        if gesture:
            self.gesture(gesture)
        self.play_file(wav_file)

    def generate_audio_to_queue(self, text: str):
        wav_file = self.generate_file(text)
        self.audio_queue.put(wav_file)

    def parallel_output_from_queue(self, emotion: str = "happy", gesture: str = None):
        # TODO: check if works and think how to track if an emotion / gesture is done or not
        """Coordinate parallel outputs"""
        wav_file = None
        while wav_file is None:
            print(self.audio_queue.empty())
            if not self.audio_queue.empty():
                wav_file = self.audio_queue.get()
            else:
                time.sleep(1)
            
        self.show_emotion(emotion, True)
        if gesture:
            self.gesture(gesture)
        self.play_file(wav_file, True)