#MANUAL_AUTO
import sys
import time
import signal
import threading
from dataclasses import dataclass
import random
import os
import json
from xxlimited import new
#from turtle import left, right

import board
import busio
import digitalio
import RPi.GPIO as GPIO
from adafruit_pca9685 import PCA9685
import adafruit_vl53l0x

# =========================================================
# LORA IMPORT
# =========================================================
sys.path.append("/home/pi/human_sound_model/pySX127x")
from SX127x.board_config import BOARD
from SX127x.LoRa import LoRa
from SX127x.constants import MODE

SENSOR_FILE = "/tmp/full_sensor.json"
SENSOR_REQ_FILE = "/tmp/sensor_request.json"

last_json_sent = ""
last_json_send_time = 0
JSON_SEND_INTERVAL = 1.0

AUTO_SCAN_INTERVAL = 9999
AUTO_STABLE_WAIT = 0.35
AUTO_SNAPSHOT_WAIT = 0.15
person_detect_count = 0
PERSON_CONFIRM_COUNT = 5
person_clear_count = 0
PERSON_CLEAR_COUNT = 10
marker_detect_armed = True
last_auto_scan_time = 0
auto_scan_busy = False

# =========================================================
# LORA GPIO CONFIG
# =========================================================
RST_PIN = 22
DIO0_PIN = 4
FREQ = 433.0

LORA_TIMEOUT = 2.0
last_lora_time = 0

def custom_setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(RST_PIN, GPIO.OUT)
    GPIO.output(RST_PIN, 1)

    GPIO.setup(DIO0_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def custom_teardown():
    try:
        GPIO.cleanup()
    except Exception:
        pass

    try:
        if BOARD.spi is not None:
            BOARD.spi.close()
            BOARD.spi = None
    except Exception:
        pass
    GPIO.cleanup()

BOARD.setup = custom_setup
BOARD.teardown = custom_teardown
BOARD.add_event_detect = lambda *args, **kwargs: None
BOARD.add_events = lambda *args, **kwargs: None

# =========================================================
# PCA9685
# =========================================================
I2C_FREQ = 100000
PCA_LEFT_ADDR = 0x40
PCA_RIGHT_ADDR = 0x60

# =========================================================
# VL53 CONFIG
# =========================================================
XSHUT_LEFT_PIN = board.D17
XSHUT_RIGHT_PIN = board.D27

MAX_DISTANCE_CM = 60
CLEAR_CM = 35
LEFT_OBSTACLE_CM = 15
RIGHT_OBSTACLE_CM = 20
DANGER_CM = 10
CONFIRM_COUNT = 2
DIFF_BALANCE_CM = 3

near_count_l = 0
near_count_r = 0

# =========================================================
# OFFSET
# =========================================================
offL = [
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0
]

offR = [
    -5, 0, 0, 0,
    0, 0, -5, 0,
    0, 0, 0, 0,
    0, 0, 0, 0
]

# =========================================================
# LEFT CHANNELS
# =========================================================
LF_COXA  = 15
LF_FEMUR = 14
LF_TIBIA = 13
LM_COXA  = 12
LM_FEMUR = 11
LM_TIBIA = 10
LR_COXA  = 9
LR_FEMUR = 8
LR_TIBIA = 7

# =========================================================
# RIGHT CHANNELS
# =========================================================
RF_COXA  = 0
RF_FEMUR = 1
RF_TIBIA = 2
RM_COXA  = 3
RM_FEMUR = 4
RM_TIBIA = 5
RR_COXA  = 6
RR_FEMUR = 7
RR_TIBIA = 8

# =========================================================
# THÔNG SỐ
# =========================================================
MID = 300

# lift khi di chuyển
L_UP_FEMUR = 420
L_UP_TIBIA = 420
R_UP_FEMUR = 185
R_UP_TIBIA = 185
RF_UP_FEMUR = 170
RF_UP_TIBIA = 170
# LOW pose đã test
L_LOW_FEMUR = 355
L_LOW_TIBIA = 265
R_LOW_FEMUR = 245
R_LOW_TIBIA = 335

# bước tiến/lùi
L_FWD = 350
L_BWD = 250
R_FWD = 250
R_BWD = 350

# quay trái/phải
L_TURN_FWD = 340
L_TURN_BWD = 260
R_TURN_FWD = 260
R_TURN_BWD = 340

# =========================================================
# SPEED
# =========================================================
GAIT_STEP_DELAY = 0.003
GAIT_STEP_SIZE = 4
GAIT_PHASE_DELAY = 0.002

TRANSITION_STEP_DELAY = 0.03
TRANSITION_STEP_SIZE = 2

STAND_HOLD = 0.0

# =========================================================
# ROBOT STATE
# =========================================================
ROBOT_BOOTING  = 0
ROBOT_LOW      = 1
ROBOT_STANDING = 2
ROBOT_MOVING   = 3

DIR_FORWARD = 0
DIR_BACKWARD = 1
DIR_TURN_LEFT = 2
DIR_TURN_RIGHT = 3

# =========================================================
# GLOBALS
# =========================================================
robotState = ROBOT_BOOTING
current_cmd = "IDLE"
running = True

pose_lock = threading.Lock()
motion_enabled = False
busy_transition = False

MODE_MANUAL = "MANUAL"
MODE_AUTO = "AUTO"

robot_mode = MODE_MANUAL
vl_left = None
vl_right = None

# =========================================================
# MARKER
# =========================================================
marker_count = 0
marker_busy = False
robot_locked = False

MARKER_PUSH_SERVO_1 = 9
MARKER_PUSH_SERVO_2 = 10
MARKER_DOOR_SERVO   = 11

SERVOMIN = 120
SERVOMAX = 600

PUSH1_HOME = 10
PUSH1_HALF = 80
PUSH1_OUT  = 155

PUSH2_HOME  = 10
PUSH2_SMALL = 80
PUSH2_OUT   = 155

DOOR_CLOSE = 72
DOOR_OPEN  = 0

# =========================================================
# STUCK DETECT + ALERT
# =========================================================
hard_block_count = 0
HARD_BLOCK_CONFIRM = 6
HARD_BLOCK_CM = 20

STUCK_TIME = 3.0
STUCK_DELTA_CM = 2.5
MAX_STUCK_RETRY = 2

stuck_start_time = None
last_stuck_left = None
last_stuck_right = None
stuck_retry_count = 0

lora_radio = None
lora_lock = threading.Lock()
last_alert_time = 0
ALERT_COOLDOWN = 5.0

# =========================================================
# I2C + PCA INIT
# =========================================================
i2c = busio.I2C(board.SCL, board.SDA, frequency=I2C_FREQ)
pcaL = PCA9685(i2c, address=PCA_LEFT_ADDR)
pcaR = PCA9685(i2c, address=PCA_RIGHT_ADDR)
pcaL.frequency = 50
pcaR.frequency = 50

# =========================================================
# POSE DATA
# =========================================================
@dataclass
class RobotPose:
    lfC: int = MID
    lfF: int = L_LOW_FEMUR
    lfT: int = L_LOW_TIBIA

    lmC: int = MID
    lmF: int = L_LOW_FEMUR
    lmT: int = L_LOW_TIBIA

    lrC: int = MID
    lrF: int = L_LOW_FEMUR
    lrT: int = L_LOW_TIBIA

    rfC: int = MID
    rfF: int = R_LOW_FEMUR
    rfT: int = R_LOW_TIBIA

    rmC: int = MID
    rmF: int = R_LOW_FEMUR
    rmT: int = R_LOW_TIBIA

    rrC: int = MID
    rrF: int = R_LOW_FEMUR
    rrT: int = R_LOW_TIBIA

pose = RobotPose()

# =========================================================
# LOW LEVEL PWM
# =========================================================
def pulse_to_duty_cycle(pulse: int) -> int:
    pulse = max(0, min(4095, pulse))
    return pulse << 4

def set_pwm(pca: PCA9685, ch: int, pulse: int):
    pca.channels[ch].duty_cycle = pulse_to_duty_cycle(pulse)

def setL(ch: int, pulse: int):
    set_pwm(pcaL, ch, pulse + offL[ch])

def setR(ch: int, pulse: int):
    set_pwm(pcaR, ch, pulse + offR[ch])

# =========================================================
# LEG SETTERS
# =========================================================
def setLF(coxa: int, femur: int, tibia: int):
    setL(LF_COXA, coxa)
    setL(LF_FEMUR, femur)
    setL(LF_TIBIA, tibia)

def setLM(coxa: int, femur: int, tibia: int):
    setL(LM_COXA, coxa)
    setL(LM_FEMUR, femur)
    setL(LM_TIBIA, tibia)

def setLR(coxa: int, femur: int, tibia: int):
    setL(LR_COXA, coxa)
    setL(LR_FEMUR, femur)
    setL(LR_TIBIA, tibia)

def setRF(coxa: int, femur: int, tibia: int):
    setR(RF_COXA, coxa)
    setR(RF_FEMUR, femur)
    setR(RF_TIBIA, tibia)

def setRM(coxa: int, femur: int, tibia: int):
    setR(RM_COXA, coxa)
    setR(RM_FEMUR, femur)
    setR(RM_TIBIA, tibia)

def setRR(coxa: int, femur: int, tibia: int):
    setR(RR_COXA, coxa)
    setR(RR_FEMUR, femur)
    setR(RR_TIBIA, tibia)

# =========================================================
# CORE ROBOT LOGIC
# =========================================================
def writeAll():
    setLF(pose.lfC, pose.lfF, pose.lfT)
    setLM(pose.lmC, pose.lmF, pose.lmT)
    setLR(pose.lrC, pose.lrF, pose.lrT)

    setRF(pose.rfC, pose.rfF, pose.rfT)
    setRM(pose.rmC, pose.rmF, pose.rmT)
    setRR(pose.rrC, pose.rrF, pose.rrT)

def moveToward(current: int, target: int, stepSize: int) -> int:
    if current < target:
        current += stepSize
        if current > target:
            current = target
    elif current > target:
        current -= stepSize
        if current < target:
            current = target
    return current

def interp_pose(updates: dict, step_size: int = GAIT_STEP_SIZE, step_delay: float = GAIT_STEP_DELAY):
    done = False
    while not done and running:
        done = True

        with pose_lock:
            for attr, target in updates.items():
                cur = getattr(pose, attr)
                new = moveToward(cur, target, step_size)
                setattr(pose, attr, new)
                if new != target:
                    done = False

            writeAll()

        time.sleep(step_delay)

def go_low_smooth():
    global robotState

    updates = {
        "lfC": MID, "lfF": L_LOW_FEMUR, "lfT": L_LOW_TIBIA,
        "lmC": MID, "lmF": L_LOW_FEMUR, "lmT": L_LOW_TIBIA,
        "lrC": MID, "lrF": L_LOW_FEMUR, "lrT": L_LOW_TIBIA,

        "rfC": MID, "rfF": R_LOW_FEMUR, "rfT": R_LOW_TIBIA,
        "rmC": MID, "rmF": R_LOW_FEMUR, "rmT": R_LOW_TIBIA,
        "rrC": MID, "rrF": R_LOW_FEMUR, "rrT": R_LOW_TIBIA,
    }

    interp_pose(
        updates,
        step_size=TRANSITION_STEP_SIZE,
        step_delay=TRANSITION_STEP_DELAY
    )
    robotState = ROBOT_LOW
def release_marker_servo(ch):
    pcaR.channels[ch].duty_cycle = 0
def standSmooth():
    global robotState

    updates = {
        "lfC": MID, "lfF": MID, "lfT": MID,
        "lmC": MID, "lmF": MID, "lmT": MID,
        "lrC": MID, "lrF": MID, "lrT": MID,

        "rfC": MID, "rfF": MID, "rfT": MID,
        "rmC": MID, "rmF": MID, "rmT": MID,
        "rrC": MID, "rrF": MID, "rrT": MID,
    }

    interp_pose(
        updates,
        step_size=TRANSITION_STEP_SIZE,
        step_delay=TRANSITION_STEP_DELAY
    )
    robotState = ROBOT_STANDING

def wake_robot():
    global robotState, motion_enabled, busy_transition, current_cmd

    if busy_transition:
        return

    if robotState == ROBOT_STANDING or robotState == ROBOT_MOVING:
        motion_enabled = True
        return

    busy_transition = True
    current_cmd = "IDLE"

    print("WAKE: LOW -> STAND")
    standSmooth()

    motion_enabled = True
    busy_transition = False

def sleep_robot():
    global robotState, motion_enabled, busy_transition, current_cmd

    if busy_transition:
        return

    busy_transition = True
    motion_enabled = False
    current_cmd = "IDLE"

    print("SLEEP: STOP -> LOW")
    go_low_smooth()

    busy_transition = False

def standFirst():
    global robotState

    if robotState == ROBOT_LOW:
        standSmooth()

        if STAND_HOLD > 0:
            time.sleep(STAND_HOLD)

def phaseA(a_lf_target: int, a_rm_target: int, a_lr_target: int,
           b_rf_target: int, b_lm_target: int, b_rr_target: int):

    interp_pose({
        "lfF": L_UP_FEMUR, "lfT": L_UP_TIBIA,
        "rmF": R_UP_FEMUR, "rmT": R_UP_TIBIA,
        "lrF": L_UP_FEMUR, "lrT": L_UP_TIBIA,
    }, step_size=GAIT_STEP_SIZE, step_delay=GAIT_STEP_DELAY)

    interp_pose({
        "lfC": a_lf_target,
        "rmC": a_rm_target,
        "lrC": a_lr_target,

        "rfC": b_rf_target,
        "lmC": b_lm_target,
        "rrC": b_rr_target,
    }, step_size=GAIT_STEP_SIZE, step_delay=GAIT_STEP_DELAY)

    interp_pose({
        "lfF": MID, "lfT": MID,
        "rmF": MID, "rmT": MID,
        "lrF": MID, "lrT": MID,
    }, step_size=GAIT_STEP_SIZE, step_delay=GAIT_STEP_DELAY)

def phaseB(b_rf_target: int, b_lm_target: int, b_rr_target: int,
           a_lf_target: int, a_rm_target: int, a_lr_target: int):

    interp_pose({
        "rfF": RF_UP_FEMUR, "rfT": RF_UP_TIBIA,
        "lmF": L_UP_FEMUR, "lmT": L_UP_TIBIA,
        "rrF": R_UP_FEMUR, "rrT": R_UP_TIBIA,
    }, step_size=GAIT_STEP_SIZE, step_delay=GAIT_STEP_DELAY)

    interp_pose({
        "rfC": b_rf_target,
        "lmC": b_lm_target,
        "rrC": b_rr_target,

        "lfC": a_lf_target,
        "rmC": a_rm_target,
        "lrC": a_lr_target,
    }, step_size=GAIT_STEP_SIZE, step_delay=GAIT_STEP_DELAY)

    interp_pose({
        "rfF": MID, "rfT": MID,
        "lmF": MID, "lmT": MID,
        "rrF": MID, "rrT": MID,
    }, step_size=GAIT_STEP_SIZE, step_delay=GAIT_STEP_DELAY)
def safe_return_mid():
    global robotState

    # Nhóm A: LF, RM, LR
    interp_pose({
        "lfF": L_UP_FEMUR, "lfT": L_UP_TIBIA,
        "rmF": R_UP_FEMUR, "rmT": R_UP_TIBIA,
        "lrF": L_UP_FEMUR, "lrT": L_UP_TIBIA,
    }, step_size=GAIT_STEP_SIZE, step_delay=GAIT_STEP_DELAY)

    time.sleep(0.05)

    interp_pose({
        "lfC": MID,
        "rmC": MID,
        "lrC": MID,
    }, step_size=2, step_delay=0.004)

    time.sleep(0.03)

    interp_pose({
        "lfF": MID, "lfT": MID,
        "rmF": MID, "rmT": MID,
        "lrF": MID, "lrT": MID,
    }, step_size=GAIT_STEP_SIZE, step_delay=GAIT_STEP_DELAY)

    time.sleep(0.05)

    # Nhóm B: RF, LM, RR
    interp_pose({
        "rfF": R_UP_FEMUR, "rfT": R_UP_TIBIA,
        "lmF": L_UP_FEMUR, "lmT": L_UP_TIBIA,
        "rrF": R_UP_FEMUR, "rrT": R_UP_TIBIA,
    }, step_size=GAIT_STEP_SIZE, step_delay=GAIT_STEP_DELAY)

    time.sleep(0.05)

    interp_pose({
        "rfC": MID,
        "lmC": MID,
        "rrC": MID,
    }, step_size=2, step_delay=0.004)

    time.sleep(0.03)

    interp_pose({
        "rfF": MID, "rfT": MID,
        "lmF": MID, "lmT": MID,
        "rrF": MID, "rrT": MID,
    }, step_size=GAIT_STEP_SIZE, step_delay=GAIT_STEP_DELAY)

    robotState = ROBOT_STANDING
def moveRobot(direction: int, steps: int, auto_stand: bool = False):
    global robotState

    if not motion_enabled:
        return

    standFirst()
    robotState = ROBOT_MOVING

    for _ in range(steps):
        if not running or not motion_enabled:
            robotState = ROBOT_STANDING
            return

        if direction == DIR_FORWARD:
            phaseA(L_FWD, R_FWD, L_FWD, R_BWD, L_BWD, R_BWD)
            time.sleep(GAIT_PHASE_DELAY)
            phaseB(R_FWD, L_FWD, R_FWD, L_BWD, R_BWD, L_BWD)
            time.sleep(GAIT_PHASE_DELAY)

        elif direction == DIR_BACKWARD:
            phaseA(L_BWD, R_BWD, L_BWD, R_FWD, L_FWD, R_FWD)
            time.sleep(GAIT_PHASE_DELAY)
            phaseB(R_BWD, L_BWD, R_BWD, L_FWD, R_FWD, L_FWD)
            time.sleep(GAIT_PHASE_DELAY)

        elif direction == DIR_TURN_LEFT:
            phaseA(
                L_TURN_BWD, R_TURN_FWD, L_TURN_BWD,
                R_TURN_BWD, L_TURN_FWD, R_TURN_BWD
            )
            time.sleep(GAIT_PHASE_DELAY)
            phaseB(
                R_TURN_FWD, L_TURN_BWD, R_TURN_FWD,
                L_TURN_FWD, R_TURN_BWD, L_TURN_FWD
            )
            time.sleep(GAIT_PHASE_DELAY)

        elif direction == DIR_TURN_RIGHT:
            phaseA(
                L_TURN_FWD, R_TURN_BWD, L_TURN_FWD,
                R_TURN_FWD, L_TURN_BWD, R_TURN_FWD
            )
            time.sleep(GAIT_PHASE_DELAY)
            phaseB(
                R_TURN_BWD, L_TURN_FWD, R_TURN_BWD,
                L_TURN_BWD, R_TURN_FWD, L_TURN_BWD
            )
            time.sleep(GAIT_PHASE_DELAY)

    if auto_stand:
        safe_return_mid()
    else:
        robotState = ROBOT_STANDING
def angle_to_pulse(angle):
    angle = max(0, min(180, angle))
    return int(SERVOMIN + (angle / 180.0) * (SERVOMAX - SERVOMIN))

def set_marker_servo(ch, angle):
    pulse = angle_to_pulse(angle)
    pcaR.channels[ch].duty_cycle = pulse_to_duty_cycle(pulse)

def marker_init():
    set_marker_servo(MARKER_PUSH_SERVO_1, PUSH1_HOME)
    set_marker_servo(MARKER_PUSH_SERVO_2, PUSH2_HOME)
    set_marker_servo(MARKER_DOOR_SERVO, DOOR_CLOSE)
    time.sleep(1)

    release_marker_servo(MARKER_PUSH_SERVO_1)
    release_marker_servo(MARKER_PUSH_SERVO_2)
    release_marker_servo(MARKER_DOOR_SERVO)
def marker_open_door():
    set_marker_servo(MARKER_DOOR_SERVO, DOOR_OPEN)
    time.sleep(0.5)
    release_marker_servo(MARKER_DOOR_SERVO)
def marker_close_door():
    set_marker_servo(MARKER_DOOR_SERVO, DOOR_CLOSE)
    time.sleep(0.5)
    release_marker_servo(MARKER_DOOR_SERVO)


def drop_marker_sequence():
    global marker_busy, marker_count

    if marker_busy:
        return

    if marker_count >= 3:
        return

    marker_busy = True
    marker_count += 1
    n = marker_count

    try:
        print(f"[MARKER] DROP MARKER {n}")

        # =========================
        # MARKER 1
        # =========================
        if n == 1:
            marker_open_door()

            set_marker_servo(MARKER_PUSH_SERVO_1, PUSH1_HALF)
            time.sleep(1.2)

            release_marker_servo(MARKER_PUSH_SERVO_1)

            marker_close_door()

        # =========================
        # MARKER 2
        # =========================
        elif n == 2:
            marker_open_door()

            set_marker_servo(MARKER_PUSH_SERVO_1, PUSH1_OUT)
            time.sleep(1.2)

            release_marker_servo(MARKER_PUSH_SERVO_1)

            set_marker_servo(MARKER_PUSH_SERVO_2, PUSH2_SMALL)
            time.sleep(1.2)

            release_marker_servo(MARKER_PUSH_SERVO_2)

            marker_close_door()

        # =========================
        # MARKER 3
        # =========================
        elif n == 3:
            marker_open_door()

            set_marker_servo(MARKER_PUSH_SERVO_2, PUSH2_OUT)
            time.sleep(1.2)

            release_marker_servo(MARKER_PUSH_SERVO_2)

            set_marker_servo(MARKER_PUSH_SERVO_2, PUSH2_HOME)
            time.sleep(1.2)

            release_marker_servo(MARKER_PUSH_SERVO_2)

            set_marker_servo(MARKER_PUSH_SERVO_1, PUSH1_HOME)
            time.sleep(1.2)

            release_marker_servo(MARKER_PUSH_SERVO_1)

            marker_close_door()

    except Exception as e:
        print("[MARKER ERROR]", e)

    finally:
        marker_busy = False

def handle_person_marker_detect():
    global robot_locked, motion_enabled, current_cmd, robot_mode

    if robot_locked:
        return

    if marker_busy:
        return

    if marker_count >= 3:
        return

    old_mode = robot_mode

    print("[PERSON] DETECTED -> STOP, LOW, DROP MARKER")

    robot_locked = True
    motion_enabled = False
    current_cmd = "IDLE"

    # Tạm thoát AUTO để auto_loop không ghi đè FORWARD
    robot_mode = MODE_MANUAL

    time.sleep(0.3)

    standSmooth()
    time.sleep(0.7)

    go_low_smooth()
    time.sleep(1.0)

    drop_marker_sequence()

    if old_mode == MODE_AUTO:
        print("[MARKER] DONE -> CONTINUE AUTO")

        global last_auto_scan_time
        last_auto_scan_time = time.time()
        robot_locked = False
        robot_mode = MODE_AUTO
        motion_enabled = True
        current_cmd = "FORWARD"
    else:
        print("[MARKER] DONE -> WAIT WAKE")

# =========================================================
# LORA RX
# =========================================================
class LoRaRx(LoRa):
    def __init__(self, verbose=False):
        super(LoRaRx, self).__init__(verbose)

        self.set_mode(MODE.SLEEP)
        time.sleep(0.05)

        self.set_dio_mapping([0] * 6)

        self.set_freq(FREQ)
        self.set_pa_config(pa_select=1)
        self.set_bw(7)
        self.set_coding_rate(1)
        self.set_spreading_factor(7)
        self.set_rx_crc(False)
        self.set_sync_word(0x12)

        self.clear_irq_flags(
            RxDone=1,
            PayloadCrcError=1,
            ValidHeader=1,
            RxTimeout=1,
            FhssChangeChannel=1,
            CadDone=1
        )

        print("LoRa RX configured")
        print("Version:", hex(self.get_version()))

    def start_rx(self):
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)

def send_lora_alert(alert_text):
    global last_alert_time, lora_radio

    now = time.time()
    if now - last_alert_time < ALERT_COOLDOWN:
        return

    if lora_radio is None:
        print("LORA ALERT FAIL: radio not ready")
        return

    msg = "ALERT:" + alert_text
    print("SEND ALERT:", msg)

    with lora_lock:
        try:
            payload = list(msg.encode("utf-8"))
            
            time.sleep(0.05)
            lora_radio.set_mode(MODE.STDBY)
            time.sleep(0.02)

            lora_radio.clear_irq_flags(TxDone=1)
            lora_radio.write_payload(payload)
            lora_radio.set_mode(MODE.TX)

            timeout = time.time() + 0.7

            while time.time() < timeout:
                irq = lora_radio.get_irq_flags()

                if irq.get("tx_done"):
                    print("ALERT TX OK")
                    lora_radio.clear_irq_flags(TxDone=1)
                    lora_radio.reset_ptr_rx()
                    lora_radio.set_mode(MODE.RXCONT)
                    last_alert_time = now
                    return

                time.sleep(0.01)

            print("ALERT TX TIMEOUT")
            lora_radio.set_mode(MODE.RXCONT)

        except Exception as e:
            print("ALERT ERROR:", e)
            try:
                lora_radio.set_mode(MODE.RXCONT)
            except Exception:
                pass

def read_sensor_json_text():
    try:
        if not os.path.exists(SENSOR_FILE):
            return None

        with open(SENSOR_FILE, "r") as f:
            data = json.load(f)

        return json.dumps(data, separators=(",", ":"))

    except Exception as e:
        print("READ SENSOR JSON ERROR:", e)
        return None

def request_sensor_snapshot():
    try:
        req = {
            "request": 1,
            "time": time.time()
        }

        with open(SENSOR_REQ_FILE, "w") as f:
            json.dump(req, f)

        print("REQUEST SENSOR SNAPSHOT")
        return True

    except Exception as e:
        print("REQUEST SENSOR ERROR:", e)
        return False

def auto_stationary_scan():
    global motion_enabled, current_cmd, last_auto_scan_time, auto_scan_busy

    if auto_scan_busy:
        return

    auto_scan_busy = True

    print("AUTO SCAN -> STOP AND SNAPSHOT")

    # Dừng robot lại
    motion_enabled = False
    current_cmd = "IDLE"

    time.sleep(0.15)

    # Chờ robot ổn định
    time.sleep(AUTO_STABLE_WAIT)

    # Yêu cầu sensor.py chốt radar + MPU6050
    request_sensor_snapshot()

    # Chờ sensor.py ghi JSON mới
    time.sleep(AUTO_SNAPSHOT_WAIT)

    # Gửi JSON sau khi đo đủ 4 chức năng
    json_text = read_sensor_json_text()
    if json_text:
        send_lora_json(json_text)

    # Cho robot chạy lại
    motion_enabled = True
    current_cmd = "FORWARD"
    last_auto_scan_time = time.time()
    auto_scan_busy = False

    print("AUTO SCAN DONE -> CONTINUE")

def send_lora_json(json_text):
    global lora_radio

    if lora_radio is None:
        return False

    if len(json_text.encode("utf-8")) > 220:
        print("JSON TOO LONG")
        return False

    with lora_lock:
        try:
            payload = list(json_text.encode("utf-8"))

            lora_radio.set_mode(MODE.STDBY)
            time.sleep(0.02)

            lora_radio.clear_irq_flags(TxDone=1)
            lora_radio.write_payload(payload)
            lora_radio.set_mode(MODE.TX)

            timeout = time.time() + 0.7

            while time.time() < timeout:
                irq = lora_radio.get_irq_flags()

                if irq.get("tx_done"):
                    print("JSON TX OK:", json_text)
                    lora_radio.clear_irq_flags(TxDone=1)
                    lora_radio.reset_ptr_rx()
                    lora_radio.set_mode(MODE.RXCONT)
                    return True

                time.sleep(0.01)

            print("JSON TX TIMEOUT")
            lora_radio.set_mode(MODE.RXCONT)
            return False

        except Exception as e:
            print("JSON TX ERROR:", e)
            try:
                lora_radio.set_mode(MODE.RXCONT)
            except Exception:
                pass
            return False

def send_lora_json_reliable(json_text, repeat=3, gap=0.3):
    ok_any = False

    for i in range(repeat):
        ok = send_lora_json(json_text)

        if ok:
            ok_any = True

        time.sleep(gap)

    return ok_any

def sensor_json_lora_loop():
    global last_json_sent, last_json_send_time
    global person_detect_count, person_clear_count, marker_detect_armed
    print("SENSOR JSON LORA LOOP START...")

    IMPORTANT_KEYS = [
        "s",
        "cp",
        "cd",
        "cc",
        "mp",
        "md",
        "r",
        "m"
    ]

    FORCE_SEND_INTERVAL = 1.5

    while running:
        now = time.time()

        json_text = read_sensor_json_text()

        # Mặc định vẫn cho phép gửi JSON
        allow_send_json = True

        # Manual đang chạy thì KHÔNG gửi JSON để tránh nghẽn LoRa
        # Nhưng vẫn đọc JSON để bắt detect người và thả marker
        if robot_mode == MODE_MANUAL:
            if current_cmd in ["FORWARD", "BACKWARD", "TURN_LEFT", "TURN_RIGHT"]:
                allow_send_json = False

            if motion_enabled:
                allow_send_json = False

        if json_text:
            important_change = False

            try:
                new = json.loads(json_text)

                cam_person = new.get("cp", 0)

                # Debug để biết JSON camera đang trả gì
                #print("[DEBUG CAM]", cam_value, cp)

                global person_detect_count

                if cam_person > 0:
                    person_detect_count += 1
                    person_clear_count = 0
                else:
                    person_detect_count = 0
                    person_clear_count += 1

                if person_clear_count >= PERSON_CLEAR_COUNT:
                    marker_detect_armed = True

                if (
                    marker_detect_armed
                    and person_detect_count >= PERSON_CONFIRM_COUNT
                    and not robot_locked
                    and marker_count < 3
                    ):
                    marker_detect_armed = False
                    person_detect_count = 0
                    person_clear_count = 0
                    handle_person_marker_detect()

                if last_json_sent:
                    old = json.loads(last_json_sent)

                    for k in IMPORTANT_KEYS:
                        if old.get(k) != new.get(k):
                            important_change = True
                            break
                else:
                    important_change = True

            except Exception as e:
                print("JSON COMPARE ERROR:", e)
                important_change = True

            if allow_send_json:
                if important_change:
                    ok = send_lora_json_reliable(json_text, repeat=2, gap=0.12)
                elif now - last_json_send_time >= FORCE_SEND_INTERVAL:
                    ok = send_lora_json(json_text)

                if ok:
                    last_json_sent = json_text
                    last_json_send_time = now

        time.sleep(0.05)

def lora_rx_loop():
    global current_cmd, last_lora_time, lora_radio, robot_mode, robot_locked

    lora = LoRaRx(verbose=False)
    lora.start_rx()
    lora_radio = lora

    print("Pi5 LoRa RX polling mode...")

    try:
        while running:
            irq = lora.get_irq_flags()

            if irq.get("rx_done"):
                payload = lora.read_payload(nocheck=True)

                try:
                    text = bytes(payload).decode("utf-8", "ignore").strip()
                except Exception:
                    text = ""

                print("Received:", text)

                if text.startswith("CMD:"):
                    cmd = text.replace("CMD:", "").strip()

                    # =========================
                    # AUTO MODE
                    # =========================
                    if cmd == "AUTO":
                        if robot_locked:
                            robot_locked = False
                            print("[RESUME] AUTO AFTER MARKER")
                        robot_mode = MODE_AUTO
                        current_cmd = "IDLE"
                        last_lora_time = time.time()
                        wake_robot()
                        print("CHUYEN SANG AUTO MODE")

                    # =========================
                    # THOAT AUTO / LENH DUNG
                    # =========================
                    elif cmd in ["STOP", "SLEEP"]:
                        robot_mode = MODE_MANUAL
                        current_cmd = cmd
                        last_lora_time = time.time()
                        print("THOAT AUTO -> MANUAL, CMD =", cmd)

                    elif cmd == "IDLE":
                        if robot_mode == MODE_AUTO:
                            print("AUTO MODE: IGNORE IDLE")
                        else:
                            current_cmd = "IDLE"
                            last_lora_time = time.time()
                            print("MANUAL CMD = IDLE")

                    # =========================
                    # MANUAL CONTROL
                    # =========================
                    elif cmd in ["FORWARD", "BACKWARD", "TURN_LEFT", "TURN_RIGHT", "WAKE"]:

                        if robot_locked and cmd != "WAKE":
                            print("ROBOT LOCKED -> IGNORE CMD", cmd)

                        else:
                            if cmd == "WAKE" and robot_locked:
                                robot_locked = False
                                print("[RESUME] WAKE AFTER MARKER")

                            robot_mode = MODE_MANUAL
                            current_cmd = cmd
                            last_lora_time = time.time()

                            print("MANUAL CMD =", cmd)

                lora.clear_irq_flags(
                    RxDone=1,
                    PayloadCrcError=1,
                    ValidHeader=1,
                    RxTimeout=1
                )

                lora.reset_ptr_rx()
                lora.set_mode(MODE.RXCONT)

            elif irq.get("payload_crc_error"):
                print("LoRa CRC ERROR")
                lora.clear_irq_flags(PayloadCrcError=1, RxDone=1)
                lora.reset_ptr_rx()
                lora.set_mode(MODE.RXCONT)

            # Timeout chỉ áp dụng cho MANUAL, không áp dụng AUTO
            if robot_mode == MODE_MANUAL:
                if last_lora_time != 0 and time.time() - last_lora_time > LORA_TIMEOUT:
                    current_cmd = "IDLE"

            time.sleep(0.01)

    finally:
        try:
            lora.set_mode(MODE.SLEEP)
        except Exception:
            pass

# =========================================================
# VL53L0X
# =========================================================
def init_vl53():
    global vl_left, vl_right

    print("Init 2 VL53L0X...")

    x1 = digitalio.DigitalInOut(XSHUT_LEFT_PIN)
    x2 = digitalio.DigitalInOut(XSHUT_RIGHT_PIN)

    x1.direction = digitalio.Direction.OUTPUT
    x2.direction = digitalio.Direction.OUTPUT

    x1.value = False
    x2.value = False
    time.sleep(0.2)

    x1.value = True
    time.sleep(0.2)
    vl_left = adafruit_vl53l0x.VL53L0X(i2c)
    vl_left.set_address(0x30)

    x2.value = True
    time.sleep(0.2)
    vl_right = adafruit_vl53l0x.VL53L0X(i2c)
    vl_right.set_address(0x31)

    print("VL53 READY: LEFT=0x30 | RIGHT=0x31")

def read_cm(sensor):
    try:
        d = sensor.range / 10.0

        if d <= 0:
            return MAX_DISTANCE_CM

        if d > MAX_DISTANCE_CM:
            return MAX_DISTANCE_CM

        return d

    except Exception:
        return MAX_DISTANCE_CM

# =========================================================
# AUTO VL53 LOOP
# =========================================================
def check_stuck(left, right, moving_cmd):
    global stuck_start_time, last_stuck_left, last_stuck_right

    if left >= CLEAR_CM and right >= CLEAR_CM:
        stuck_start_time = None
        last_stuck_left = left
        last_stuck_right = right
        return False
    
    if moving_cmd not in ["FORWARD", "BACKWARD", "TURN_LEFT", "TURN_RIGHT"]:
        stuck_start_time = None
        last_stuck_left = left
        last_stuck_right = right
        return False

    if last_stuck_left is None or last_stuck_right is None:
        last_stuck_left = left
        last_stuck_right = right
        stuck_start_time = None
        return False

    diff_l = abs(left - last_stuck_left)
    diff_r = abs(right - last_stuck_right)

    if diff_l < STUCK_DELTA_CM and diff_r < STUCK_DELTA_CM:
        if stuck_start_time is None:
            stuck_start_time = time.time()

        if time.time() - stuck_start_time >= STUCK_TIME:
            return True
    else:
        stuck_start_time = None

    last_stuck_left = left
    last_stuck_right = right
    return False


def reset_stuck_detect():
    global stuck_start_time, last_stuck_left, last_stuck_right

    stuck_start_time = None
    last_stuck_left = None
    last_stuck_right = None

def auto_loop():
    global turn_until_time
    global current_cmd, motion_enabled, robot_mode, stuck_retry_count
    global near_count_l, near_count_r
    global hard_block_count, HARD_BLOCK_CONFIRM
    global last_auto_scan_time
    global robot_locked, marker_busy

    print("AUTO VL53 START...")
    time.sleep(3)

    wake_robot()
    motion_enabled = True
    last_auto_scan_time = time.time()

    last_decision = "FORWARD"
    same_decision_count = 0

    while running:
        if robot_mode != MODE_AUTO:
            time.sleep(0.1)
            continue

        # =========================
        # MARKER BUSY
        # =========================
        if robot_locked or marker_busy:
            current_cmd = "IDLE"
            motion_enabled = False
            time.sleep(0.1)
            continue

        if time.time() - last_auto_scan_time >= AUTO_SCAN_INTERVAL:
            auto_stationary_scan()
            time.sleep(0.1)
            continue
        
        left = read_cm(vl_left)
        right = read_cm(vl_right)
# Nếu vật cản đã biến mất thì hủy lệnh quay/lùi cũ
        if left >= CLEAR_CM and right >= CLEAR_CM:
            current_cmd = "FORWARD"
            reset_stuck_detect()
        # =========================
        # HARD BLOCK DETECT
        # =========================
        if left < HARD_BLOCK_CM and right < HARD_BLOCK_CM:
            hard_block_count += 1
        else:
            hard_block_count = 0

        if hard_block_count >= HARD_BLOCK_CONFIRM:
            stuck_retry_count += 1
            print("HARD BLOCK! RETRY =", stuck_retry_count)

            hard_block_count = 0
            reset_stuck_detect()

            if stuck_retry_count <= MAX_STUCK_RETRY:
                print("HARD BLOCK -> BACKWARD")

                current_cmd = "BACKWARD"
                time.sleep(1.2)

                current_cmd = "TURN_RIGHT"
                print("HARD BLOCK -> TURN_RIGHT")
                time.sleep(0.6)

                auto_stationary_scan()

                last_decision = "FORWARD"
                same_decision_count = 0

                continue
            else:
                print("HARD BLOCK -> STOP + ALERT")

                current_cmd = "STOP"
                robot_mode = MODE_MANUAL
                send_lora_alert("STUCK")

                stuck_retry_count = 0
                time.sleep(1.0)

                continue

        # ===== NEAR DETECT =====
        if left < LEFT_OBSTACLE_CM and right >= RIGHT_OBSTACLE_CM:
            near_count_l += 1
        else:
            near_count_l = 0

        if right < RIGHT_OBSTACLE_CM and left >= LEFT_OBSTACLE_CM:
            near_count_r += 1
        else:
            near_count_r = 0

        # =========================
        # DECISION (FINAL CLEAN)
        # =========================
        if left < DANGER_CM and right < DANGER_CM:
            decision = "BACKWARD"

        elif left < LEFT_OBSTACLE_CM and right >= RIGHT_OBSTACLE_CM:
            if near_count_l >= 1:
                decision = "TURN_RIGHT"
            else:
                decision = "FORWARD"

        elif right < RIGHT_OBSTACLE_CM and left >= LEFT_OBSTACLE_CM:
            if near_count_r >= 1:
                decision = "TURN_LEFT"
            else:
                decision = "FORWARD"

        elif left < CLEAR_CM or right < CLEAR_CM:
            if left < right:
                decision = "TURN_RIGHT"
            else:
                decision = "TURN_LEFT"

        else:
            decision = "FORWARD"

        # =========================
        # DEBOUNCE
        # =========================
        if decision == last_decision:
            same_decision_count += 1
        else:
            same_decision_count = 1
            last_decision = decision

        # =========================
        # APPLY COMMAND
        # =========================
        if decision == "FORWARD":
            current_cmd = "FORWARD"

        elif decision == "BACKWARD":
            current_cmd = "BACKWARD"

        elif same_decision_count >= CONFIRM_COUNT:
            current_cmd = decision

        else:
            current_cmd = "FORWARD"
        
        # =========================
        # STUCK DETECT
        # =========================
        if check_stuck(left, right, current_cmd):
            stuck_retry_count += 1
            print("ROBOT STUCK! RETRY =", stuck_retry_count)

            reset_stuck_detect()

            if stuck_retry_count <= MAX_STUCK_RETRY:
                print("STUCK -> AUTO BACKWARD")
                current_cmd = "BACKWARD"
                time.sleep(1.0)

            else:
                print("STUCK HARD -> STOP + ALERT")
                current_cmd = "STOP"
                robot_mode = MODE_MANUAL
                send_lora_alert("STUCK")
                stuck_retry_count = 0
                time.sleep(1.0)

        else:
            if current_cmd == "FORWARD":
                stuck_retry_count = 0

        print(
            f"L={left:.1f} R={right:.1f} | "
            f"DECISION={decision} COUNT={same_decision_count} | "
            f"CMD={current_cmd}"
        )
        
        time.sleep(0.25)

# =========================================================
# THREAD ĐIỀU KHIỂN ROBOT
# =========================================================
def motion_loop():
    global current_cmd, robotState, robot_mode, motion_enabled

    last_cmd = "IDLE"

    while running:
        cmd = current_cmd

        if busy_transition:
            time.sleep(0.02)
            last_cmd = cmd
            continue

        if cmd == "WAKE":
            wake_robot()
            time.sleep(0.05)

        elif cmd == "SLEEP":
            sleep_robot()
            time.sleep(0.05)

        elif cmd == "FORWARD":
            wake_robot()
            moveRobot(DIR_FORWARD, 2, auto_stand=False)

        elif cmd == "BACKWARD":
            wake_robot()
            moveRobot(DIR_BACKWARD, 2, auto_stand=False)

        elif cmd == "TURN_LEFT":
            wake_robot()
            moveRobot(DIR_TURN_LEFT, 1, auto_stand=False)

        elif cmd == "TURN_RIGHT":
            wake_robot()
            moveRobot(DIR_TURN_RIGHT, 1, auto_stand=False)

        elif cmd == "STOP":
            motion_enabled = False
            current_cmd = "IDLE"
            safe_return_mid()
            print("STOP -> MANUAL | REQUEST SENSOR")

            # báo cho sensor.py chốt dữ liệu radar + MPU lúc robot đứng yên
            request_sensor_snapshot()

            # đợi sensor.py cập nhật file JSON
            time.sleep(0.8)

            robot_mode = MODE_MANUAL

            time.sleep(0.05)
        else:
            time.sleep(0.02)

        last_cmd = cmd

# =========================================================
# CLEANUP
# =========================================================
def cleanup(*_):
    global running, motion_enabled
    running = False
    motion_enabled = False

    print("\nDang dung robot an toan...")

    try:
        go_low_smooth()
        with pose_lock:
            writeAll()
    except Exception as e:
        print("Cleanup error:", e)

    try:
        pcaL.deinit()
        pcaR.deinit()
    except Exception:
        pass

    try:
        BOARD.teardown()
    except Exception:
        pass

    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

# =========================================================
# MAIN
# =========================================================
def main():
    global robotState, motion_enabled, last_lora_time, current_cmd

    print("Khoi dong hexapod Pi 5 LoRa RX...")
    
    time.sleep(1.0)

    BOARD.setup()
    init_vl53()

    with pose_lock:
        writeAll()

    time.sleep(0.5)

    go_low_smooth()
    marker_init()
    motion_enabled = False
    current_cmd = "IDLE"
    last_lora_time = time.time()

    print("Robot dang o LOW. Cho lenh WAKE tu Pi3...")

    t_motion = threading.Thread(target=motion_loop, daemon=True)
    t_motion.start()

    t_lora = threading.Thread(target=lora_rx_loop, daemon=True)
    t_lora.start()

    t_auto = threading.Thread(target=auto_loop, daemon=True)
    t_auto.start()

    t_json = threading.Thread(target=sensor_json_lora_loop, daemon=True)
    t_json.start()

    while running:
        time.sleep(1)
if __name__ == "__main__":
    main()