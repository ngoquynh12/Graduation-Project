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
import logging

# ======= Disable ALL logging from LD2410 package =======
for name in logging.Logger.manager.loggerDict.keys():
    if "LD2410" in name or "ld2410" in name:
        logging.getLogger(name).disabled = True

logging.getLogger().setLevel(logging.CRITICAL)


# ===== ADD LED =====
from gpiozero import LED
led = LED(23)   # <-- LED tr�n GPIO23

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

# RADAR FLAGS
radar_detected = False
radar_distance = 0.0
moving_count = 0
still_count = 0

# ======= SIGNAL HANDLER =======
def handle_exit(sig=None, frame=None):
    global running
    print("\nExiting program...")
    running = False
    led.off()
    time.sleep(0.3)
    sys.exit(0)

signal.signal(signal.SIGINT, handle_exit)

# =====================================================
# ================ LOAD MODELS ========================
# =====================================================
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
            # ================ CAMERA STREAM SECTION ===============
# =====================================================
def camera_stream():
    global image_detected, detected_label
    print("Starting camera stream...")

    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (1280, 720), "format": "RGB888"},
        controls={"FrameRate": 15}
    )
    picam2.configure(config)
    picam2.start()

    last_infer = time.time()

    try:
        while running:
            frame = picam2.capture_array()
            now = time.time()

            if now - last_infer > 0.25:
                last_infer = now

                infer_frame = cv2.resize(frame, (IMG_SIZE, IMG_SIZE))

                results = yolo_model.predict(
                    infer_frame, imgsz=IMG_SIZE, conf=CONF_THRESHOLD, verbose=False
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
                    elif label in ["dog", "cat"]:
                        detected_label = label
                        image_detected = True
                        break

                if image_detected:
                    print(f"[CAMERA] {detected_label.capitalize()} detected")

                annotated = results[0].plot()

            else:
                annotated = frame

            small_view = cv2.resize(annotated, (640, 360))
            cv2.imshow("YOLO Camera", small_view)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                handle_exit()

    except Exception as e:
        print("Camera error:", e)

    finally:
        picam2.stop()
        cv2.destroyAllWindows()

# =====================================================
# ================== RADAR LOOP =======================
# =====================================================
from LD2410 import ld2410, ld2410_consts

SERIAL_PORT = "/dev/ttyAMA0"
MOVING_TH = 20
HOLD_ON = 5
HOLD_OFF = 10

def radar_loop():
    global radar_detected, radar_distance, moving_count, still_count

    try:
        radar = ld2410.LD2410(
            port=SERIAL_PORT,
            baud_rate=ld2410_consts.PARAM_DEFAULT_BAUD
        )
        print("[RADAR] LD2410 connected.")
    except:
        print("[RADAR] ERROR connecting LD2410")
        return

    while running:
        frame, _, _ = radar.get_radar_data()

        if frame:
            moving_dist = frame[1] / 100.0
            moving_energy = frame[2]

            radar_distance = moving_dist

            if moving_energy >= MOVING_TH:
                moving_count += 1
                still_count = 0
            else:
                still_count += 1
                moving_count = 0

            if moving_count >= HOLD_ON:
                radar_detected = True

            if still_count >= HOLD_OFF:
                radar_detected = False

        time.sleep(0.1)# ================== FUSION LOOP ======================
# =====================================================
def fusion_loop():
    global detected_label
    last_state = ""

    while running:
        state = "[CLEAR] No detection"

        human_cam = (detected_label == "human" and image_detected)
        human_audio = sound_detected
        human_radar = radar_detected

        # ===== OR-FUSION: 1 TRONG 3 L� �? =====
        if human_cam or human_audio or human_radar:

            led.on()   # <-- B?T LED

            if human_cam:
                state = "[CAMERA] Human detected"

            if human_radar:
                state = f"[RADAR] Movement detected at {radar_distance:.2f} m"

            if human_audio:
                state = "[AUDIO] Human voice detected"

        else:
            led.off()  # <-- T?T LED
            state = "[CLEAR] No detection"

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
    Thread(target=radar_loop, daemon=True).start()
    Thread(target=fusion_loop, daemon=True).start()

    while running:
        time.sleep(1)