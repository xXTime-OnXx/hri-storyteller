#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse
import subprocess

def handle_play_wav(req):
    wav_path = "/root/catkin_ws/hello.wav"
    try:
        subprocess.Popen(["aplay", wav_path])  # "aplay" is built-in for WAV files
        return TriggerResponse(success=True, message=f"Playing {wav_path}")
    except Exception as e:
        return TriggerResponse(success=False, message=str(e))

if __name__ == "__main__":
    rospy.init_node("wav_player_service")
    srv = rospy.Service("/play_wav", Trigger, handle_play_wav)
    rospy.loginfo("Ready to play WAV file on Qt PC.")
    rospy.spin()