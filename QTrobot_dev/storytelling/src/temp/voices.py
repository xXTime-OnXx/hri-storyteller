#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import json
from TTS.api import TTS
import os
import tempfile

# Initialize ROS node
rospy.init_node('qt_tts_player', anonymous=True)
audio_pub = rospy.Publisher("/qt_robot/audio/playWav", String, queue_size=10)
talk_pub = rospy.Publisher("/qt_robot/behavior/talkText", String, queue_size=10)
rospy.sleep(1)  # wait for publishers to register

# Initialize TTS
tts = TTS(model_name="tts_models/en/ljspeech/tacotron2-DDC")  # You can pick other voices

def play_tts(text):
    # Generate temporary WAV file
    tmp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".wav")
    tmp_file.close()
    tts.tts_to_file(text=text, file_path=tmp_file.name)

    # Send audio to QTrobot
    msg = {"file": tmp_file.name}
    audio_pub.publish(String(data=json.dumps(msg)))

    # Also send text for lip-sync
    talk_pub.publish(String(data=text))

    # Wait a bit to let QTrobot play audio
    rospy.sleep(3 + len(text.split()) / 2)  # rough estimate based on word count

    # Optional: delete temp file
    os.remove(tmp_file.name)

if __name__ == "__main__":
    story_text = "Hello, I'm your storyteller! This is a custom voice on QTrobot."
    play_tts(story_text)
    rosp