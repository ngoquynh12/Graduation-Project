#SENSOR
import os
import cv2
import json
import math
import sys
import time
import signal
import logging
import smbus
from threading import Thread, Lock
from collections import Counter

from ultralytics import YOLO
from picamera2 import Picamera2
from gpiozero import LED
import RPi.GPIO as GPIO
from LD2410 import ld2410, ld2410_consts

# =====================================================
# CONFIG
# =====================================================
SENSOR_FILE = "/tmp/full_sensor.json"
SENSOR_REQ_FILE = "/tmp/sensor_request.json"

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

# request STOP snapshot
last_sensor_request_time = 0
need_snapshot = False

# giá trị radar + MPU đã chốt khi robot đứng yên
last_stable_rad = 0
last_stable_dst = 0.0
last_stable_mmi = "I"
last_stable_pga = 0.0

# audio shared
mic_person = 0
mic_dog = 0
mic_noise = 0
mic_state = "no sound"

# =====================================================
# EXIT
# =====================================================
def handle_exit(sig=None, frame=None):
    global running
    print("\nExiting...")
    running = False
    led.off()
    time.sleep(1.0)
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
            #cv2.imshow("YOLO Camera", show)

            #if cv2.waitKey(1) & 0xFF == ord("q"):
                #running = False

            #time.sleep(0.001)

    finally:
        picam2.stop()
        #cv2.destroyAllWindows()

# =====================================================
# RADAR
# =====================================================
def radar_loop():
    global radar_detected, radar_distance, moving_count, still_count

    radar = None

    try:
        radar = ld2410.LD2410(SERIAL_PORT, ld2410_consts.PARAM_DEFAULT_BAUD)
        print("[RADAR] Connected.")

        while running:
            try:
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

            except Exception as e:
                print("[RADAR] read error:", e)
                radar_detected = False
                radar_distance = 0.0
                time.sleep(0.5)

            time.sleep(0.1)

    except Exception as e:
        print("[RADAR] ERROR connecting LD2410:", e)

    finally:
        print("[RADAR] Closing UART...")
        try:
            if radar is not None:
                if hasattr(radar, "serial"):
                    radar.serial.close()
                elif hasattr(radar, "ser"):
                    radar.ser.close()
                elif hasattr(radar, "uart"):
                    radar.uart.close()
                else:
                    del radar
        except Exception as e:
            print("[RADAR] close error:", e)

# =====================================================
# MPU6050 NEW
# =====================================================
MPU_ADDR = 0x68
MPU_BUS = 1
bus = smbus.SMBus(MPU_BUS)

PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B

ACCEL_SCALE = 16384.0
MPU_DT = 0.05
CALIB_SAMPLES = 100
MEASURE_SAMPLES = 40

ALPHA = 0.90
NOISE_FLOOR_GAL = 1.0
prev_ax = None
prev_ay = None
prev_az = None

accel_offset = [0.0, 0.0, 0.0]

filtered_ax = 0.0
filtered_ay = 0.0
filtered_az = 0.0

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

def mpu_write(reg, value):
    bus.write_byte_data(MPU_ADDR, reg, value)

def mpu_read_word(reg):
    high = bus.read_byte_data(MPU_ADDR, reg)
    low = bus.read_byte_data(MPU_ADDR, reg + 1)

    value = (high << 8) | low

    if value >= 0x8000:
        value = -((65535 - value) + 1)

    return value

def mpu_init():
    print("[MPU] Initializing MPU6050...")

    mpu_write(PWR_MGMT_1, 0x00)
    time.sleep(0.1)

    mpu_write(SMPLRT_DIV, 0x09)
    mpu_write(CONFIG, 0x04)
    mpu_write(ACCEL_CONFIG, 0x00)

    print("[MPU] Ready.")

def read_raw_accel():
    raw_ax = mpu_read_word(ACCEL_XOUT_H)
    raw_ay = mpu_read_word(ACCEL_XOUT_H + 2)
    raw_az = mpu_read_word(ACCEL_XOUT_H + 4)
    return raw_ax, raw_ay, raw_az

def convert_accel(raw_ax, raw_ay, raw_az):
    ax = raw_ax / ACCEL_SCALE
    ay = raw_ay / ACCEL_SCALE
    az = raw_az / ACCEL_SCALE
    return ax, ay, az

def low_pass(new_value, old_value):
    return ALPHA * old_value + (1.0 - ALPHA) * new_value

def mpu_calibrate_when_stable():
    global accel_offset
    global prev_ax, prev_ay, prev_az

    print("[MPU] Calibrating at STOP...")

    sx = sy = sz = 0.0

    for _ in range(CALIB_SAMPLES):
        raw_ax, raw_ay, raw_az = read_raw_accel()
        ax, ay, az = convert_accel(raw_ax, raw_ay, raw_az)

        sx += ax
        sy += ay
        sz += az

        time.sleep(MPU_DT)

    accel_offset[0] = sx / CALIB_SAMPLES
    accel_offset[1] = sy / CALIB_SAMPLES
    accel_offset[2] = (sz / CALIB_SAMPLES) - 1.0

    print(
        f"[MPU] Offset ax={accel_offset[0]:.4f}, "
        f"ay={accel_offset[1]:.4f}, "
        f"az={accel_offset[2]:.4f}"
    )
    prev_ax = None
    prev_ay = None
    prev_az = None

def measure_pga_mmi():
    global filtered_ax, filtered_ay, filtered_az

    # Mồi filter bằng mẫu đầu tiên sau calibration
    raw_ax, raw_ay, raw_az = read_raw_accel()
    ax, ay, az = convert_accel(raw_ax, raw_ay, raw_az)

    ax -= accel_offset[0]
    ay -= accel_offset[1]
    az -= accel_offset[2]

    filtered_ax = ax
    filtered_ay = ay
    filtered_az = az

    max_pga = 0.0
    prev_mx = filtered_ax
    prev_my = filtered_ay
    prev_mz = filtered_az

    for _ in range(MEASURE_SAMPLES):
        raw_ax, raw_ay, raw_az = read_raw_accel()
        ax, ay, az = convert_accel(raw_ax, raw_ay, raw_az)

        ax -= accel_offset[0]
        ay -= accel_offset[1]
        az -= accel_offset[2]

        filtered_ax = low_pass(ax, filtered_ax)
        filtered_ay = low_pass(ay, filtered_ay)
        filtered_az = low_pass(az, filtered_az)

        dx = filtered_ax - prev_mx
        dy = filtered_ay - prev_my
        dz = filtered_az - prev_mz

        dynamic_g = math.sqrt(dx*dx + dy*dy + dz*dz)
        pga = dynamic_g * 980.665

        prev_mx = filtered_ax
        prev_my = filtered_ay
        prev_mz = filtered_az

        if pga < NOISE_FLOOR_GAL:
            pga = 0.0

        if pga > max_pga:
            max_pga = pga

        time.sleep(MPU_DT)

    level, category = pga_to_mmi(max_pga)

    return max_pga, level, category

def vibration_loop():
    global pga_value, mmi_level, mmi_category
    global vibration_detected, prev_mmi
    global filtered_ax, filtered_ay, filtered_az
    global prev_ax, prev_ay, prev_az

    try:
        mpu_init()
        print("[MPU] Waiting robot startup...")
        time.sleep(18)

        print("[MPU] Startup calibration...")
        mpu_calibrate_when_stable()
    except Exception as e:
        print("[MPU INIT ERROR]", e)
        return
    
    raw_ax, raw_ay, raw_az = read_raw_accel()
    ax, ay, az = convert_accel(raw_ax, raw_ay, raw_az)

    ax -= accel_offset[0]
    ay -= accel_offset[1]
    az -= accel_offset[2]

    filtered_ax = ax
    filtered_ay = ay
    filtered_az = az

    print("[MPU] Continuous PGA/MMI monitoring started...")  
    while running:
        try:
            raw_ax, raw_ay, raw_az = read_raw_accel()
            ax, ay, az = convert_accel(raw_ax, raw_ay, raw_az)

            ax -= accel_offset[0]
            ay -= accel_offset[1]
            az -= accel_offset[2]

            filtered_ax = low_pass(ax, filtered_ax)
            filtered_ay = low_pass(ay, filtered_ay)
            filtered_az = low_pass(az, filtered_az)

            if prev_ax is None:
                prev_ax = filtered_ax
                prev_ay = filtered_ay
                prev_az = filtered_az
                pga_value = 0.0
            else:
                dx = filtered_ax - prev_ax
                dy = filtered_ay - prev_ay
                dz = filtered_az - prev_az

                dynamic_g = math.sqrt(dx*dx + dy*dy + dz*dz)
                pga_value = dynamic_g * 980.665

                prev_ax = filtered_ax
                prev_ay = filtered_ay
                prev_az = filtered_az

            if pga_value < NOISE_FLOOR_GAL:
                pga_value = 0.0

            mmi_level, mmi_category = pga_to_mmi(pga_value)
            vibration_detected = pga_value > 5

            if prev_mmi != mmi_level:
                print(
                    f"[VIBRATION] PGA={pga_value:.2f} gal | "
                    f"MMI={mmi_level} - {mmi_category}"
                )
                prev_mmi = mmi_level

            time.sleep(MPU_DT)

        except Exception as e:
            print("[MPU ERROR]", e)
            time.sleep(0.5)  

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

        if os.getenv("TERM"):
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
    global current_fusion_state, last_stable_pga
    last = ""

    while running:
        human_cam = image_detected and detected_label == "person"
        human_audio = mic_person == 1
        human_radar = radar_detected
        quake = last_stable_pga > 5

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
# SENSOR REQUEST FROM MANUAL_AUTO
# =====================================================
def sensor_request_loop():
    global last_sensor_request_time, need_snapshot

    while running:
        try:
            if os.path.exists(SENSOR_REQ_FILE):
                with open(SENSOR_REQ_FILE, "r") as f:
                    data = json.load(f)

                req_time = float(data.get("time", 0))

                if req_time != last_sensor_request_time:
                    last_sensor_request_time = req_time
                    need_snapshot = True
                    print("[REQ] STOP snapshot requested")

        except Exception as e:
            print("[REQ ERROR]", e)

        time.sleep(0.1)

# =====================================================
# WRITE JSON
# =====================================================
def write_sensor_data_loop():
    global need_snapshot
    global last_stable_rad, last_stable_dst
    global last_stable_mmi, last_stable_pga
    global pga_value, mmi_level, mmi_category, vibration_detected

    while running:
        try:
            with state_lock:
                state = current_fusion_state

            counts = Counter(detected_labels.copy())

            # Camera + Audio luôn lấy giá trị mới nhất
            cam_now = 1 if image_detected else 0
            cam_person_now = int(counts.get("human", 0) + counts.get("person", 0))
            cam_dog_now = int(counts.get("dog", 0))
            cam_cat_now = int(counts.get("cat", 0))

            mic_person_now = int(mic_person)
            mic_dog_now = int(mic_dog)
            mic_noise_now = int(mic_noise)

            if need_snapshot:
                print("[SNAPSHOT] Robot stopped -> wait stable")
                time.sleep(2.0)

                try:
                    mpu_calibrate_when_stable()
                    pga_value, mmi_level, mmi_category = measure_pga_mmi()
                    vibration_detected = pga_value > 5
                except Exception as e:
                    print("[MPU SNAPSHOT ERROR]", e)

                last_stable_rad = 1 if radar_detected else 0
                last_stable_dst = round(radar_distance, 2)
                last_stable_mmi = mmi_level
                last_stable_pga = round(pga_value, 2)

                need_snapshot = False
                print("[SNAPSHOT] RADAR + MPU updated")

            packet_state = "CLEAR"

            if cam_person_now > 0:
                packet_state = "CAMERA"

            elif mic_person_now == 1:
                packet_state = "AUDIO"

            elif last_stable_rad == 1:
                packet_state = "RADAR"

            elif last_stable_pga > 5:
                packet_state = "VIBRATION"
            
            packet = {
                #"cam": cam_now,
                "cp": cam_person_now,
                "cd": cam_dog_now,
                "cc": cam_cat_now,

                "mp": mic_person_now,
                "md": mic_dog_now,
                #"mn": mic_noise_now,

                "r": last_stable_rad,
                "d": last_stable_dst,

                "m": last_stable_mmi,
                "p": round(last_stable_pga, 2),

                "s": packet_state
            }

            tmp_file = "/tmp/full_sensor.json.tmp"

            with open(tmp_file, "w") as f:
                json.dump(packet, f, separators=(",", ":"))

            os.replace(tmp_file, "/tmp/full_sensor.json")

        except Exception as e:
            print("[JSON ERROR]", e)

        time.sleep(0.2)

# =====================================================
# MAIN
# =====================================================
if __name__ == "__main__":
    
    Thread(target=audio_state_loop, daemon=True).start()
    Thread(target=camera_stream, daemon=True).start()
    Thread(target=radar_loop, daemon=True).start()
    Thread(target=vibration_loop, daemon=True).start()
    Thread(target=fusion_loop, daemon=True).start()
    Thread(target=sensor_request_loop, daemon=True).start()
    Thread(target=write_sensor_data_loop, daemon=True).start()
    Thread(target=print_dashboard, daemon=True).start()

    while running:
        time.sleep(1)