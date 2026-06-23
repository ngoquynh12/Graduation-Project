import subprocess
import librosa
import numpy as np

DEVICE = "plughw:2,0"

print("Recording...")
subprocess.run([
    "arecord",
    "-D", DEVICE,
    "-f", "S16_LE",
    "-r", "16000",
    "-c", "2",
    "-d", "1",
    "debug.wav"
], check=True)

audio, sr = librosa.load("debug.wav", sr=16000, mono=False)

print("sr =", sr)
print("shape =", audio.shape)

if audio.ndim == 1:
    channels = [audio]
else:
    channels = [audio[0], audio[1]]

for i, ch in enumerate(channels):
    ch = ch.astype(np.float32)
    print(f"\n=== Channel {i} ===")
    print("min      =", np.min(ch))
    print("max      =", np.max(ch))
    print("mean     =", np.mean(ch))
    print("mean_abs =", np.mean(np.abs(ch)))
    print("std      =", np.std(ch))
    print("first20  =", ch[:20])