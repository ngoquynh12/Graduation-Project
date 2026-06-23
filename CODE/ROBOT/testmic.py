import os
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"

import json
import time
import librosa
import signal
import subprocess
import numpy as np
import tensorflow as tf
from collections import deque, Counter

AUDIO_MODEL_PATH = "/home/pi/human_sound_model/best_model.keras"
AUDIO_DEVICE = "plughw:2,0"
STATE_FILE = "/home/pi/human_sound_model/audio_state.json"

MODEL_SAMPLE_RATE = 16000
MIC_SAMPLE_RATE = 16000

WINDOW_DURATION = 1.0
STEP_DURATION = 0.25

WINDOW_SAMPLES = int(MODEL_SAMPLE_RATE * WINDOW_DURATION)
STEP_SAMPLES = int(MODEL_SAMPLE_RATE * STEP_DURATION)

N_FFT = 1024
HOP_LENGTH = 256
N_MELS = 64

DOG_IDX = 0
HUMAN_IDX = 1
NOISE_IDX = 2

DOG_THRESHOLD = 0.60
HUMAN_THRESHOLD = 0.35
NOISE_THRESHOLD = 0.95

HUMAN_NOISE_MARGIN = 0.12
DOG_HUMAN_MARGIN = 0.12


CLASSES = ["dog", "human", "noise"]

ENERGY_THRESHOLD = 0.000001

VOTE_SIZE = 3
DOG_MIN_VOTES = 1
HUMAN_MIN_VOTES = 2
NOISE_MIN_VOTES = 3

recent_preds = deque(maxlen=VOTE_SIZE)
CHANNELS = 2
BYTES_PER_SAMPLE = 2
CHUNK_BYTES = STEP_SAMPLES * CHANNELS * BYTES_PER_SAMPLE

audio_buffer = np.zeros(WINDOW_SAMPLES, dtype=np.float32)
running = True



def handle_exit(sig=None, frame=None):
    global running
    running = False
    print("\n[AUDIO] Exiting...")

signal.signal(signal.SIGINT, handle_exit)

print("[AUDIO] Loading model...")
if not os.path.exists(AUDIO_MODEL_PATH):
    raise FileNotFoundError(f"Không tìm thấy audio model: {AUDIO_MODEL_PATH}")
audio_model = tf.keras.models.load_model(AUDIO_MODEL_PATH)
print("[AUDIO] Model ready.")

def write_state(label):
    data = {
        "mic_state": label,
        "mic_person": 1 if label == "human" else 0,
        "mic_dog": 1 if label == "dog" else 0,
        "mic_noise": 1 if label == "noise" else 0,
        "ts": time.time(),
    }

    tmp_file = STATE_FILE + ".tmp"
    with open(tmp_file, "w", encoding="utf-8") as f:
        json.dump(data, f)
    os.replace(tmp_file, STATE_FILE)

def raw_stats(audio):
    x = audio.astype(np.float32)
    x = x - np.mean(x)
    rms = float(np.sqrt(np.mean(x ** 2)))
    peak = float(np.max(np.abs(x)))
    return rms, peak

def audio_to_logmel(audio):
    if len(audio) > WINDOW_SAMPLES:
        audio = audio[:WINDOW_SAMPLES]
    elif len(audio) < WINDOW_SAMPLES:
        audio = np.pad(audio, (0, WINDOW_SAMPLES - len(audio)))

    x = audio.astype(np.float32)
    x = x - np.mean(x)

    peak = np.max(np.abs(x))
    if peak > 1e-6:
        x = x / peak

    mel = librosa.feature.melspectrogram(
        y=x,
        sr=MODEL_SAMPLE_RATE,
        n_fft=N_FFT,
        hop_length=HOP_LENGTH,
        n_mels=N_MELS,
    )

    logmel = librosa.power_to_db(mel, ref=np.max)
    return logmel.astype(np.float32)

def predict_audio(audio):
    feature = audio_to_logmel(audio)
    feature = feature[np.newaxis, ..., np.newaxis]
    pred = audio_model.predict(feature, verbose=0)[0]
    return pred
def print_prediction(pred, instant_label, final_label, reason, energy):
    dog_p = float(pred[DOG_IDX])
    human_p = float(pred[HUMAN_IDX])
    noise_p = float(pred[NOISE_IDX])

    print(
        f"[AUDIO] final={final_label} | instant={instant_label} | "
        f"human={human_p*100:.1f}% | dog={dog_p*100:.1f}% | noise={noise_p*100:.1f}% | "
        f"energy={energy:.6f} | reason={reason}"
    )


def start_arecord():
    cmd = [
        "arecord",
        "-D", AUDIO_DEVICE,
        "-f", "S16_LE",
        "-r", str(MIC_SAMPLE_RATE),
        "-c", str(CHANNELS),
        "-t", "raw",
    ]
    return subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        bufsize=CHUNK_BYTES * 4,
    )

def read_exact(stream, nbytes):
    data = bytearray()
    while len(data) < nbytes:
        chunk = stream.read(nbytes - len(data))
        if not chunk:
            return None
        data.extend(chunk)
    return bytes(data)

def raw_to_mono_ch0(raw_bytes):
    audio_i16 = np.frombuffer(raw_bytes, dtype=np.int16)
    if audio_i16.size == 0:
        return None

    audio_i16 = audio_i16.reshape(-1, CHANNELS)
    ch0 = audio_i16[:, 0].astype(np.float32) / 32768.0
    return ch0
def decide_label(audio, pred):
    energy = float(np.mean(audio ** 2))

    dog_p = float(pred[DOG_IDX])
    human_p = float(pred[HUMAN_IDX])
    noise_p = float(pred[NOISE_IDX])

    if energy < ENERGY_THRESHOLD:
        return -1, "too_low_energy"

    # Dog: chỉ cần dog cao và hơn human đủ xa
    if dog_p >= DOG_THRESHOLD and dog_p - human_p >= DOG_HUMAN_MARGIN:
        return DOG_IDX, "dog_strong"

    # Human: phải rõ hơn noise, không nhận kiểu fallback nữa
    if human_p >= HUMAN_THRESHOLD and human_p >= noise_p:
        return HUMAN_IDX, "human_ok"

    # Noise: chỉ nhận khi noise rất cao
    if noise_p >= 0.95 and dog_p < 0.50 and human_p < 0.30:
        return NOISE_IDX, "noise_strong"

    return -1, "uncertain"
def smooth(label_idx):
    recent_preds.append(label_idx)

    valid = [p for p in recent_preds if p != -1]
    if not valid:
        return "uncertain"

    counts = Counter(valid)

    dog_votes = counts.get(DOG_IDX, 0)
    human_votes = counts.get(HUMAN_IDX, 0)
    noise_votes = counts.get(NOISE_IDX, 0)

    if dog_votes >= DOG_MIN_VOTES:
        return "dog"

    if human_votes >= HUMAN_MIN_VOTES:
        return "human"

    if noise_votes >= NOISE_MIN_VOTES:
        return "noise"

    return "uncertain"
def audio_loop():
    global running, audio_buffer

    proc = None
    last_print = ""

    try:
        proc = start_arecord()
        print("[AUDIO] Realtime audio started.")

        while running:
            raw = read_exact(proc.stdout, CHUNK_BYTES)
            if raw is None:
                print("[AUDIO] Không đọc được audio.")
                time.sleep(0.2)
                continue

            chunk = raw_to_mono_ch0(raw)
            if chunk is None:
                continue

            if len(chunk) > STEP_SAMPLES:
                chunk = chunk[:STEP_SAMPLES]
            elif len(chunk) < STEP_SAMPLES:
                chunk = np.pad(chunk, (0, STEP_SAMPLES - len(chunk)))

            audio_buffer[:-STEP_SAMPLES] = audio_buffer[STEP_SAMPLES:]
            audio_buffer[-STEP_SAMPLES:] = chunk

            pred = predict_audio(audio_buffer)

            decided_idx, reason = decide_label(audio_buffer, pred)
            instant_label = "uncertain" if decided_idx == -1 else CLASSES[decided_idx]

            final_label = smooth(decided_idx)

            energy = float(np.mean(audio_buffer ** 2))

            write_state(final_label)

            if final_label != last_print:
                print_prediction(pred, instant_label, final_label, reason, energy)
                last_print = final_label

    finally:
        if proc is not None:
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except Exception:
                pass

if __name__ == "__main__":
    write_state("no sound")
    audio_loop()