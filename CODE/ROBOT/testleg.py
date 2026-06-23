#MANUAL
import sys
import time
import signal
import threading
from dataclasses import dataclass

import board
import busio
import RPi.GPIO as GPIO
from adafruit_pca9685 import PCA9685

# =========================================================
# LORA IMPORT
# =========================================================
sys.path.append("/home/pi/human_sound_model/pySX127x")
from SX127x.board_config import BOARD
from SX127x.LoRa import LoRa
from SX127x.constants import MODE

# =========================================================
# LORA GPIO CONFIG
# =========================================================
RST_PIN = 22
DIO0_PIN = 26
FREQ = 433.0

LORA_TIMEOUT = 0.5
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
# OFFSET
# =========================================================
offL = [0, 0, 0, 0, 0, 0, 0, 0, 0]
offR = [-5, 0, 0, 0, 0, 0, -5, 0, 0]

# =========================================================
# LEFT CHANNELS
# =========================================================
LF_COXA  = 0
LF_FEMUR = 1
LF_TIBIA = 2
LM_COXA  = 3
LM_FEMUR = 4
LM_TIBIA = 5
LR_COXA  = 6
LR_FEMUR = 7
LR_TIBIA = 8

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
L_UP_FEMUR = 375
L_UP_TIBIA = 375
R_UP_FEMUR = 230
R_UP_TIBIA = 230

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
GAIT_STEP_DELAY = 0.012
GAIT_STEP_SIZE = 3
GAIT_PHASE_DELAY = 0.012

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

    if robotState != ROBOT_STANDING:
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
        "rfF": R_UP_FEMUR, "rfT": R_UP_TIBIA,
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

def moveRobot(direction: int, steps: int, auto_stand: bool = False):
    global robotState

    if not motion_enabled:
        return

    standFirst()
    robotState = ROBOT_MOVING

    for _ in range(steps):
        if not running or not motion_enabled:
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
        standSmooth()

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

def lora_rx_loop():
    global current_cmd, last_lora_time

    lora = LoRaRx(verbose=False)
    lora.start_rx()

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

                    if cmd in [
                        "FORWARD", "BACKWARD", "TURN_LEFT",
                        "TURN_RIGHT", "STOP", "WAKE", "SLEEP", "IDLE"
                    ]:
                        current_cmd = cmd
                        last_lora_time = time.time()
                        print("LORA CMD =", cmd)

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

            if last_lora_time != 0 and time.time() - last_lora_time > LORA_TIMEOUT:
                current_cmd = "IDLE"

            time.sleep(0.01)

    finally:
        try:
            lora.set_mode(MODE.SLEEP)
        except Exception:
            pass

# =========================================================
# THREAD ĐIỀU KHIỂN ROBOT
# =========================================================
def motion_loop():
    global current_cmd, robotState

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
            moveRobot(DIR_FORWARD, 1, auto_stand=False)

        elif cmd == "BACKWARD":
            moveRobot(DIR_BACKWARD, 1, auto_stand=False)

        elif cmd == "TURN_LEFT":
            moveRobot(DIR_TURN_LEFT, 1, auto_stand=False)

        elif cmd == "TURN_RIGHT":
            moveRobot(DIR_TURN_RIGHT, 1, auto_stand=False)

        elif cmd == "STOP":
            time.sleep(0.02)

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

    with pose_lock:
        writeAll()

    time.sleep(0.5)

    go_low_smooth()
    motion_enabled = False
    current_cmd = "IDLE"
    last_lora_time = time.time()

    print("Robot dang o LOW. Cho lenh WAKE tu Pi3...")

    t_motion = threading.Thread(target=motion_loop, daemon=True)
    t_motion.start()

    t_lora = threading.Thread(target=lora_rx_loop, daemon=True)
    t_lora.start()

    while running:
        time.sleep(1)
if __name__ == "__main__":
    main()