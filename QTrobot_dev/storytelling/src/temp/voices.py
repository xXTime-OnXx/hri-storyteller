import pyttsx3
import os
from pydub import AudioSegment
import subprocess

text = "Hello Timon! Do you like this new voice?"
wav_file = "/root/catkin_ws/src/hello.wav"

# Initialize TTS engine
engine = pyttsx3.init()

# Set voice (male)
voices = engine.getProperty('voices')
for voice in voices:
    if "male" in voice.name.lower():  # simple filter for male voices
        engine.setProperty('voice', voice.id)
        break

# Set speech rate (slower for storytelling feel)
engine.setProperty('rate', 140)  # default is ~200

# Save to WAV
engine.save_to_file(text, wav_file)
engine.runAndWait()

# Play WAV using ROS service
subprocess.run(["rosservice", "call", "/qt_robot/audio/play_local_file", f"'{wav_file}'"])