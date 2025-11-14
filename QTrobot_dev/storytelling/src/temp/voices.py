from TTS.api import TTS
import subprocess
import time
import tempfile

text = "Once upon a time, a little fox lived at the edge of a great forest. Every day, it watched the birds flying high and dreamed of adventures. One morning, the fox decided to explore beyond the trees and discovered a sparkling river. There, it met a wise old turtle who shared stories of the forest’s secrets. From that day on, the little fox felt braver and more curious than ever, ready for new adventures each day."

# Generate a unique WAV filename
wav_file = f"/root/catkin_ws/src/audios/speak_file.wav"

# Load Coqui VCTK multi-speaker model
tts = TTS(model_name="tts_models/en/vctk/vits", progress_bar=False, gpu=False)

# Choose a male speaker (p233 = deep male)
speaker_id = "p230"

# Generate WAV at 16kHz
tts.tts_to_file(
    text=text,
    speaker=speaker_id,
    file_path=wav_file,
    audio_config={"sample_rate": 16000}
)

# --- Post-process to adjust speed ---
# Create a temporary WAV file for faster playback
speed = 0.88  # 1.0 = normal, >1.0 = faster, <1.0 = slower
fast_wav = tempfile.NamedTemporaryFile(suffix=".wav").name

subprocess.run([
    "ffmpeg", "-y", "-i", wav_file,
    "-filter:a", f"atempo={speed}",
    fast_wav
], check=True)

# Play the adjusted WAV using ROS service
subprocess.run([
    "rosservice", "call",
    "/qt_robot/audio/play_local_file",
    fast_wav
])