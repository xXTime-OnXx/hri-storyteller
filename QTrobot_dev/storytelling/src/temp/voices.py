from TTS.api import TTS
import subprocess
import time

start_time = time.time()

text = "Test, Test, I'm only Testing!"

# Final WAV filename (will contain adjusted speed and volume)
wav_file = "/root/catkin_ws/src/audios/test.wav"

# Load Coqui TTS model
tts = TTS(model_name="tts_models/en/vctk/vits", progress_bar=False, gpu=False)
model_loaded = time.time()
print(f"Model Load: {model_loaded - start_time}")

# Choose a male speaker
speaker_id = "p230"

# Generate WAV at 16kHz directly to the output file
tts.tts_to_file(
    text=text,
    speaker=speaker_id,
    file_path=wav_file,
    audio_config={"sample_rate": 16000}
)
generated_file = time.time()
print(f"Generate File: {generated_file - start_time}")

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

speed_changed = time.time()
print(f"Speed Changed: {speed_changed - start_time}")

# Play the final adjusted file
subprocess.run([
    "rosservice", "call",
    "/qt_robot/audio/play_local_file",
    wav_file
])


played_file = time.time()
print(f"Play File: {played_file - start_time}")