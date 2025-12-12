#!/usr/bin/env python3

import rospy

from generate_audio.srv import GenerateAudioFile
import subprocess

from TTS.api import TTS


class GenerateAudioFileServer():

    def __init__(self):
        rospy.init_node('generate_audio_file_server', anonymous=True)
        self.subscriber = rospy.Subscriber('qt_robot/audio/generate', GenerateAudioFile, self.generate_callback)

        self.tts = TTS(model_name="tts_models/en/vctk/vits", progress_bar=False, gpu=False)
        print(f"TTS Model Loaded")

        # Choose a male speaker
        self.speaker_id = "p230"

        rospy.spin()

    def generate_callback(self, req):
        rospy.loginfo("received request to generate tts: " + req.filename)

        filename = req.filename
        text = req.text

        wav_file = f"/root/catkin_ws/src/audios/{filename}.wav"

        try:
            self.tts.tts_to_file(
                text=text,
                speaker=self.speaker_id,
                file_path=wav_file,
                audio_config={"sample_rate": 16000}
            )
        except:
            rospy.logerr('TTS failed')
            return False


        try:
            # --- Adjust speed and volume in-place ---
            speed = 0.85  # 1.0 = normal, >1.0 = faster, <1.0 = slower
            tmp_file = wav_file.replace(".wav", ".tmp.wav")  # proper WAV extension

            subprocess.run([
                "ffmpeg", "-y", "-i", wav_file,
                "-filter:a", f"volume=5dB,atempo={speed}",
                tmp_file
            ], check=True)

            # Replace original file with adjusted version
            subprocess.run(["mv", tmp_file, wav_file], check=True)
        except:
            rospy.logerr('TTS Post-Processing failed')
            return False


        return True


if __name__ == "__main__":
    GenerateAudioFileServer()