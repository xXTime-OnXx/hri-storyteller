import pyttsx3
import os

output_folder = "/root/catkin_ws/src/QTrobot_dev/storytelling/src/temp"
os.makedirs(output_folder, exist_ok=True)

engine = pyttsx3.init()

# List and choose a voice
voices = engine.getProperty('voices')
engine.setProperty('voice', voices[1].id)  # pick a different voice

# Optional: adjust rate and volume
engine.setProperty('rate', 100)
engine.setProperty('volume', 0.8)

# Save TTS to WAV (pyttsx3 does not support MP3 directly)
output_file = os.path.join(output_folder, "hello.wav")
engine.save_to_file("Hello from QTrobot with a new voice!", output_file)
engine.runAndWait()

print(f"Saved WAV to {output_file}")