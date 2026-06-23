import os
import cv2
import json
import math
import time
import signal
import logging
from threading import Thread, Lock
from collections import Counter

from ultralytics import YOLO
from picamera2 import Picamera2
from mpu6050 import mpu6050
from gpiozero import LED
import RPi.GPIO as GPIO

import sys
sys.path.append("/home/pi/human_sound_model/pySX127x")

from SX127x.board_config import BOARD
from SX127x.LoRa import LoRa
from SX127x.constants import MODE
from LD2410 import ld2410, ld2410_consts

# =====================================================
# CONFIG
# =====================================================
STATE_FILE = "/home/pi/human_sound_model/audio_state.json"
YOLO_MODEL_PATH = "/home/pi/human_sound_model/best.pt"

CAM_W, CAM_H = 1640, 1232
SHOW_W, SHOW_H = 640, 480
IMG_SIZE = 320
CONF_THRESHOLD = 0.45
INFER_INTERVAL = 0.20

MOVING_TH = 20
HOLD_ON = 5
HOLD_OFF = 10

G = 9.80665
SERIAL_PORT = "/dev/ttyAMA0"

running = True
led = LED(23)

detected_labels = []
image_detected = False
detected_label = None

radar_detected = False
radar_distance = 0.0
moving_count = 0
still_count = 0

pga_value = 0.0
mmi_level = "I"
mmi_category = "Instrumental"
vibration_detected = False
prev_mmi = None
prev_total = None

state_lock = Lock()
current_fusion_state = "CLEAR"

# audio shared
mic_person = 0
mic_dog = 0
mic_noise = 0
mic_state = "no sound"

# =====================================================
# LORA
# =====================================================
LORA_RST_PIN = 22
LORA_DIO0_PIN = 26

def lora_custom_setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(LORA_RST_PIN, GPIO.OUT)
    GPIO.output(LORA_RST_PIN, 1)
    GPIO.setup(LORA_DIO0_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def lora_custom_teardown():
    try:
        GPIO.cleanup([LORA_RST_PIN, LORA_DIO0_PIN])
    except Exception:
        pass

    try:
        if BOARD.spi is not None:
            BOARD.spi.close()
            BOARD.spi = None
    except Exception:
        pass

BOARD.setup = lora_custom_setup
BOARD.teardown = lora_custom_teardown
BOARD.add_event_detect = lambda *args, **kwargs: None
BOARD.add_events = lambda *args, **kwargs: None

class LoRaTxPolling(LoRa):
    def __init__(self, verbose=False):
        super(LoRaTxPolling, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([0] * 6)
        self.set_freq(433.0)
        self.set_pa_config(pa_select=1)
        self.set_bw(7)
        self.set_coding_rate(1)
        self.set_spreading_factor(7)
        self.set_rx_crc(True)
        self.set_sync_word(0x12)
        self.set_agc_auto_on(True)
        self.set_low_data_rate_optim(False)

    def send_text(self, text, timeout=2.0):
        try:
            payload = text.encode("utf-8")
            if len(payload) > 220:
                return False

            self.set_mode(MODE.STDBY)
            self.clear_irq_flags(TxDone=1)
            self.write_payload(list(payload))
            self.set_mode(MODE.TX)

            t0 = time.time()
            while True:
                irq = self.get_irq_flags()
                if irq["tx_done"]:
                    self.clear_irq_flags(TxDone=1)
                    self.set_mode(MODE.STDBY)
                    return True

                if time.time() - t0 > timeout:
                    self.set_mode(MODE.STDBY)
                    return False

                time.sleep(0.02)
        except Exception:
            return False

def handle_exit(sig=None, frame=None):
    global running
    print("\nExiting...")
    running = False
    led.off()
    time.sleep(0.3)
    sys.exit(0)

signal.signal(signal.SIGINT, handle_exit)

# =====================================================
# LOAD MODEL
# =====================================================
print("Loading YOLO...")
yolo_model = YOLO(YOLO_MODEL_PATH)
print("YOLO model ready.")

# =====================================================
# AUDIO STATE READER
# =====================================================
def audio_state_loop():
    global mic_person, mic_dog, mic_noise, mic_state

    while running:
        try:
            if os.path.exists(STATE_FILE):
                with open(STATE_FILE, "r", encoding="utf-8") as f:
                    data = json.load(f)

                mic_state = data.get("mic_state", "no sound")
                mic_person = int(data.get("mic_person", 0))
                mic_dog = int(data.get("mic_dog", 0))
                mic_noise = int(data.get("mic_noise", 0))
            else:
                mic_state = "no sound"
                mic_person = mic_dog = mic_noise = 0

        except Exception as e:
            print("[AUDIO STATE ERROR]", e)

        time.sleep(0.2)

# =====================================================
# CAMERA
# =====================================================
def camera_stream():
    global running, image_detected, detected_label

    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (CAM_W, CAM_H), "format": "RGB888"},
        raw={"size": (CAM_W, CAM_H)}
    )
    picam2.configure(config)
    picam2.start()

    last_infer = 0.0
    last_annotated = None
    last_labels_key = ""

    try:
        while running:
            frame = picam2.capture_array()
            frame_small = cv2.resize(frame, (SHOW_W, SHOW_H))
            frame_bgr = frame_small

            now = time.time()
            if now - last_infer >= INFER_INTERVAL:
                last_infer = now

                results = yolo_model.predict(
                    frame_bgr, imgsz=IMG_SIZE, conf=CONF_THRESHOLD, verbose=False
                )

                image_detected = False
                detected_label = None
                detected_labels.clear()

                r0 = results[0]
                if r0.boxes is not None and len(r0.boxes) > 0:
                    confs = r0.boxes.conf.cpu().numpy()
                    clss = r0.boxes.cls.cpu().numpy().astype(int)

                    for i in range(len(confs)):
                        label = r0.names.get(int(clss[i]), str(int(clss[i])))
                        detected_labels.append(label)

                    image_detected = True

                    if "human" in detected_labels or "person" in detected_labels:
                        detected_label = "person"
                    elif "dog" in detected_labels:
                        detected_label = "dog"
                    elif "cat" in detected_labels:
                        detected_label = "cat"
                    else:
                        detected_label = detected_labels[0]

                    labels_key = ",".join(sorted(detected_labels))
                    if labels_key != last_labels_key:
                        last_labels_key = labels_key
                        info = []
                        for i in range(len(confs)):
                            label = r0.names.get(int(clss[i]), str(int(clss[i])))
                            info.append(f"{label}({float(confs[i]):.2f})")
                        print("[CAM] " + ", ".join(info), flush=True)
                else:
                    if last_labels_key != "":
                        last_labels_key = ""
                        print("[CAM] No detection", flush=True)

                last_annotated = r0.plot()

            show = last_annotated if last_annotated is not None else frame_bgr
            cv2.imshow("YOLO Camera", show)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                running = False

            time.sleep(0.001)

    finally:
        picam2.stop()
        cv2.destroyAllWindows()

# =====================================================
# RADAR
# =====================================================
def radar_loop():
    global radar_detected, radar_distance, moving_count, still_count

    try:
        radar = ld2410.LD2410(SERIAL_PORT, ld2410_consts.PARAM_DEFAULT_BAUD)
        print("[RADAR] Connected.")
    except Exception:
        print("[RADAR] ERROR connecting LD2410")
        return

    while running:
        frame, _, _ = radar.get_radar_data()

        if frame:
            dist_m = frame[1] / 100.0
            energy = frame[2]
            radar_distance = dist_m

            if energy >= MOVING_TH:
                moving_count += 1
                still_count = 0
            else:
                still_count += 1
                moving_count = 0

            radar_detected = moving_count >= HOLD_ON
            if still_count >= HOLD_OFF:
                radar_detected = False

        time.sleep(0.1)

# =====================================================
# MPU
# =====================================================
MPU = mpu6050(0x68)

def mpu_calibrate(samples=200):
    print("Calibrating MPU6050...")
    sx = sy = sz = 0
    for _ in range(samples):
        d = MPU.get_accel_data()
        sx += d["x"]
        sy += d["y"]
        sz += d["z"]
        time.sleep(0.005)

    return sx/samples, sy/samples, (sz/samples - G)

ox, oy, oz = mpu_calibrate()

MMI_TABLE = [
    (0, 1, "I", "Instrumental"),
    (1, 2, "II", "Very Weak"),
    (2, 5, "III", "Weak"),
    (5, 10, "IV", "Light"),
    (10, 25, "V", "Moderate"),
    (25, 50, "VI", "Strong"),
    (50, 100, "VII", "Very Strong"),
    (100, 250, "VIII", "Severe"),
]

def pga_to_mmi(pga):
    for lo, hi, lvl, name in MMI_TABLE:
        if lo <= pga < hi:
            return lvl, name
    return "IX+", "Extreme"

def vibration_loop():
    global prev_total, pga_value, mmi_level, mmi_category
    global vibration_detected, prev_mmi

    ax_buf, ay_buf, az_buf = [], [], []

    def smooth(v, buf, size=10):
        buf.append(v)
        if len(buf) > size:
            buf.pop(0)
        return sum(buf)/len(buf)

    while running:
        raw = MPU.get_accel_data()

        ax = smooth((raw["x"] - ox)/G, ax_buf)
        ay = smooth((raw["y"] - oy)/G, ay_buf)
        az = smooth((raw["z"] - oz)/G, az_buf)

        total = math.sqrt(ax*ax + ay*ay + az*az)

        if prev_total is None:
            prev_total = total

        dynamic = abs(total - prev_total)
        prev_total = total

        pga_value = dynamic * 980.665
        mmi_level, mmi_category = pga_to_mmi(pga_value)

        vibration_detected = pga_value > 5

        if prev_mmi != mmi_level:
            print(f"[VIBRATION] PGA={pga_value:.2f} gal | MMI={mmi_level} - {mmi_category}")
            prev_mmi = mmi_level

        time.sleep(0.1)

# =====================================================
# DASHBOARD
# =====================================================
def print_dashboard():
    while running:
        if image_detected and detected_labels:
            counts = Counter(detected_labels)
            cam_text = ", ".join(f"{k}:{v}" for k, v in counts.items())
            cam = f"CAMERA   : {cam_text}"
        else:
            cam = "CAMERA   : No detection"

        mic = f"MIC      : {mic_state}"
        rad = f"RADAR    : Movement {radar_distance:.2f} m" if radar_detected else "RADAR    : No target"
        vib = f"VIBRATION: MMI={mmi_level}  PGA={pga_value:.2f} gal"

        os.system("clear")
        print(cam)
        print(mic)
        print(rad)
        print(vib)

        time.sleep(0.15)

# =====================================================
# FUSION
# =====================================================
def fusion_loop():
    global current_fusion_state
    last = ""

    while running:
        human_cam = image_detected and detected_label == "person"
        human_audio = mic_person == 1
        human_radar = radar_detected
        quake = vibration_detected

        state = "CLEAR"

        if human_cam or human_audio or human_radar or quake:
            led.on()

            if human_cam:
                state = "CAMERA"
            elif human_audio:
                state = "AUDIO"
            elif human_radar:
                state = "RADAR"
            elif quake:
                state = "VIBRATION"
        else:
            led.off()
            state = "CLEAR"

        with state_lock:
            current_fusion_state = state

        pretty = {
            "CLEAR": "[CLEAR] No detection",
            "CAMERA": "[CAMERA] Human detected",
            "AUDIO": "[AUDIO] Human voice detected",
            "RADAR": f"[RADAR] Movement {radar_distance:.2f} m",
            "VIBRATION": f"[VIBRATION] MMI={mmi_level}"
        }[state]

        if pretty != last:
            print(pretty)
            last = pretty

        time.sleep(0.2)

# =====================================================
# LORA
# =====================================================
def build_lora_packet():
    with state_lock:
        state = current_fusion_state

    counts = Counter(detected_labels) if image_detected and detected_labels else Counter()

    packet = {
        "cam": 1 if image_detected else 0,
        "cam_person": int(counts.get("human", 0) + counts.get("person", 0)),
        "cam_dog": int(counts.get("dog", 0)),
        "cam_cat": int(counts.get("cat", 0)),
        "mic_person": int(mic_person),
        "mic_dog": int(mic_dog),
        "mic_noise": int(mic_noise),
        "rad": 1 if radar_detected else 0,
        "dst": round(radar_distance, 2),
        "mmi": mmi_level,
        "pga": round(pga_value, 2),
        "st": state
    }

    return json.dumps(packet, separators=(",", ":"))

def lora_loop():
    BOARD.setup()
    lora = None

    last_sent = ""
    last_send_time = 0.0
    HEARTBEAT_INTERVAL = 2.0

    try:
        lora = LoRaTxPolling(verbose=False)

        while running:
            try:
                msg = build_lora_packet()
                now = time.time()

                need_send = False
                if msg != last_sent:
                    need_send = True
                elif now - last_send_time >= HEARTBEAT_INTERVAL:
                    need_send = True

                if need_send:
                    print("[LORA TX]", msg)
                    ok = lora.send_text(msg)

                    if ok:
                        last_sent = msg
                        last_send_time = now
                    else:
                        print("[LORA] send failed")

                time.sleep(0.2)

            except Exception as e:
                print("[LORA LOOP ERROR]", e)
                time.sleep(0.5)

    except Exception as e:
        print("[LORA INIT ERROR]", e)

    finally:
        try:
            if lora is not None:
                lora.set_mode(MODE.SLEEP)
        except Exception:
            pass
        BOARD.teardown()

# =====================================================
# MAIN
# =====================================================
if __name__ == "__main__":
    Thread(target=audio_state_loop, daemon=True).start()
    Thread(target=camera_stream, daemon=True).start()
    Thread(target=radar_loop, daemon=True).start()
    Thread(target=vibration_loop, daemon=True).start()
    Thread(target=fusion_loop, daemon=True).start()
    Thread(target=print_dashboard, daemon=True).start()
    Thread(target=lora_loop, daemon=True).start()

    while running:
        time.sleep(1)