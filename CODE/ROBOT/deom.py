import os
import cv2
import numpy as np
import sounddevice as sd
from threading import Thread
from queue import Queue
from edge_impulse_linux.runner import ImpulseRunner
from ultralytics import YOLO
from picamera2 import Picamera2
import time
import contextlib, io, sys, signal

# ===== CONFIG =====
EI_MODEL_PATH = "/home/pi/human_sound_model/onlyhuman-linux-aarch64-v2.eim"
YOLO_MODEL_PATH = "/home/pi/human_sound_model/best.pt"
SAMPLING_RATE = 16000
CHUNK_DURATION = 0.25
BUFFER_DURATION = 2.0
IMG_SIZE = 320
CONF_THRESHOLD = 0.45
# ===================

# ----- GLOBAL FLAGS -----
running = True
sound_detected = False
image_detected = False
detected_label = None  # "human", "dog", "cat"

# ======= SIGNAL HANDLER =======
def handle_exit(sig=None, frame=None):
    global running
    print("\nExiting program...")
    running = False
    time.sleep(0.3)
    sys.exit(0)

signal.signal(signal.SIGINT, handle_exit)

# ======= LOAD MODELS =======
print("Loading Edge Impulse model...")
ei_runner = ImpulseRunner(EI_MODEL_PATH)
ei_info = ei_runner.init()
print(f"Edge model: {ei_info['project']['name']}")
print("Labels:", ei_info['model_parameters']['labels'])

print("Loading YOLO model...")
yolo_model = YOLO(YOLO_MODEL_PATH)
print("YOLO model ready.\n")

# =====================================================
# ================ AUDIO STREAM SECTION ================
# =====================================================
audio_q = Queue()
audio_buffer = np.zeros(int(BUFFER_DURATION * SAMPLING_RATE), dtype=np.int16)

def audio_callback(indata, frames, time_info, status):
    global audio_buffer
    if status:
        print("Audio status:", status)
    indata = (indata[:, 0] * 32767).astype(np.int16)
    audio_buffer = np.roll(audio_buffer, -len(indata))
    audio_buffer[-len(indata):] = indata
    if audio_q.qsize() < 3:
        audio_q.put(audio_buffer.copy())

def classify_audio():
    global sound_detected
    while running:
        if not audio_q.empty():
            data = audio_q.get()
            try:
                buf = io.StringIO()
                with contextlib.redirect_stdout(buf):
                    result = ei_runner.classify(data)
                if "result" in result and "classification" in result["result"]:
                    scores = result["result"]["classification"]
                    score_human = float(scores.get("human", 0.0))
                    if score_human > 0.7:
                        sound_detected = True
                        print(f"[AUDIO] Human voice detected ({score_human:.2f})")
                    else:
                        sound_detected = False
            except Exception as e:
                print("Audio classify error:", e)
        time.sleep(0.05)

def start_audio_stream():
    print("Starting audio stream...")
    Thread(target=classify_audio, daemon=True).start()
    try:
        with sd.InputStream(
            device=None,
            channels=1,
            dtype='float32',
            samplerate=SAMPLING_RATE,
            blocksize=int(CHUNK_DURATION * SAMPLING_RATE),
            callback=audio_callback
        ):
            while running:
                time.sleep(0.1)
    except Exception as e:
        print("Microphone init error:", e)
        print("Try running 'arecord -l' to check your mic device index.")
        while running:
            time.sleep(1)
            # =====================================================
# ================ CAMERA STREAM SECTION ===============
# =====================================================
def camera_stream():
    global image_detected, detected_label
    print("Starting camera stream...")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (320, 240), "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()
    last_infer = time.time()

    try:
        while running:
            frame = picam2.capture_array()
            now = time.time()

            if now - last_infer > 0.2:
                last_infer = now
                results = yolo_model.predict(
                    frame, imgsz=IMG_SIZE, conf=CONF_THRESHOLD, verbose=False
                )
                names = results[0].names
                classes = (
                    results[0].boxes.cls.tolist()
                    if results[0].boxes and results[0].boxes.cls is not None
                    else []
                )

                detected_label = None
                image_detected = False

                for c in classes:
                    label = names[int(c)].lower()
                    if label in ["person", "human"]:
                        detected_label = "human"
                        image_detected = True
                        break
                    elif label == "dog":
                        detected_label = "dog"
                        image_detected = True
                        break
                    elif label == "cat":
                        detected_label = "cat"
                        image_detected = True
                        break

                if image_detected:
                    print(f"[CAMERA] {detected_label.capitalize()} detected")

                annotated = results[0].plot()
                cv2.imshow("YOLO Camera", annotated)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    handle_exit()
            else:
                cv2.imshow("YOLO Camera", frame)
                cv2.waitKey(1)

    except Exception as e:
        print("Camera error:", e)
    finally:
        picam2.stop()
        cv2.destroyAllWindows()

# =====================================================
# ================== FUSION LOOP ======================
# =====================================================
def fusion_loop():
    global detected_label
    last_state = ""
    while running:
        state = "[CLEAR]"
        # Fusion only for human (audio + video)
        if detected_label == "human" and image_detected and sound_detected:
            state = "[FUSION] HUMAN DETECTED (Audio + Video)"
        elif detected_label == "human" and image_detected:
            state = "[CAMERA] Human detected"
        elif detected_label in ["dog", "cat"] and image_detected:
            state = f"[CAMERA] {detected_label.capitalize()} detected"

        if state != last_state:
            print(state)
            last_state = state
        time.sleep(0.2)

# =====================================================
# ==================== MAIN ===========================
# =====================================================
if __name__ == "__main__":
    Thread(target=start_audio_stream, daemon=True).start()
    Thread(target=camera_stream, daemon=True).start()
    Thread(target=fusion_loop, daemon=True).start()

    while running:
        time.sleep(1)