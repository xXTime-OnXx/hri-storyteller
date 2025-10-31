import pyttsx3
import os
import subprocess

# Example folder
output_folder = "/root/catkin_ws/src/QTrobot_dev/storytelling/src/temp"
os.makedirs(output_folder, exist_ok=True)  # create folder if it doesn't exist

engine = pyttsx3.init()
output_file = os.path.join(output_folder, "hello.mp3")
engine.save_to_file("Hello from QTrobot!", output_file)
engine.runAndWait()

print(f"Saved WAV to {output_file}")