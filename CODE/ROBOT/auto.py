#AUTO
import sys
import time
import signal
import threading
from dataclasses import dataclass

import board
import busio
import digitalio
import RPi.GPIO as GPIO

import adafruit_vl53l0x
from adafruit_pca9685 import PCA9685

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
# SERVO PARAMETERS
# =========================================================
MID = 300

L_UP_FEMUR = 375
L_UP_TIBIA = 375
R_UP_FEMUR = 230
R_UP_TIBIA = 230

L_LOW_FEMUR = 355
L_LOW_TIBIA = 265
R_LOW_FEMUR = 245
R_LOW_TIBIA = 335

L_FWD = 350
L_BWD = 250
R_FWD = 250
R_BWD = 350

L_TURN_FWD = 340
L_TURN_BWD = 260
R_TURN_FWD = 260
R_TURN_BWD = 340

# =========================================================
# SPEED
# =========================================================
GAIT_STEP_DELAY = 0.005
GAIT_STEP_SIZE = 4
GAIT_PHASE_DELAY = 0.005

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

vl_left = None
vl_right = None

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

    interp_pose(updates, step_size=TRANSITION_STEP_SIZE, step_delay=TRANSITION_STEP_DELAY)
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

    interp_pose(updates, step_size=TRANSITION_STEP_SIZE, step_delay=TRANSITION_STEP_DELAY)
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
    })

    interp_pose({
        "lfC": a_lf_target,
        "rmC": a_rm_target,
        "lrC": a_lr_target,

        "rfC": b_rf_target,
        "lmC": b_lm_target,
        "rrC": b_rr_target,
    })

    interp_pose({
        "lfF": MID, "lfT": MID,
        "rmF": MID, "rmT": MID,
        "lrF": MID, "lrT": MID,
    })

def phaseB(b_rf_target: int, b_lm_target: int, b_rr_target: int,
           a_lf_target: int, a_rm_target: int, a_lr_target: int):

    interp_pose({
        "rfF": R_UP_FEMUR, "rfT": R_UP_TIBIA,
        "lmF": L_UP_FEMUR, "lmT": L_UP_TIBIA,
        "rrF": R_UP_FEMUR, "rrT": R_UP_TIBIA,
    })

    interp_pose({
        "rfC": b_rf_target,
        "lmC": b_lm_target,
        "rrC": b_rr_target,

        "lfC": a_lf_target,
        "rmC": a_rm_target,
        "lrC": a_lr_target,
    })

    interp_pose({
        "rfF": MID, "rfT": MID,
        "lmF": MID, "lmT": MID,
        "rrF": MID, "rrT": MID,
    })

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
def auto_loop():
    global current_cmd, motion_enabled
    global near_count_l, near_count_r

    print("AUTO VL53 START...")
    time.sleep(3)

    wake_robot()
    motion_enabled = True

    last_decision = "FORWARD"
    same_decision_count = 0

    while running:
        left = read_cm(vl_left)
        right = read_cm(vl_right)

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

        print(
            f"L={left:.1f} R={right:.1f} | "
            f"DECISION={decision} COUNT={same_decision_count} | "
            f"CMD={current_cmd}"
        )

        time.sleep(0.25)

# =========================================================
# MOTION LOOP
# =========================================================
def motion_loop():
    global current_cmd, robotState

    while running:
        cmd = current_cmd

        if busy_transition:
            time.sleep(0.02)
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
            moveRobot(DIR_BACKWARD, 2, auto_stand=False)

        elif cmd == "TURN_LEFT":
            moveRobot(DIR_TURN_LEFT, 2, auto_stand=False)

        elif cmd == "TURN_RIGHT":
            moveRobot(DIR_TURN_RIGHT, 2, auto_stand=False)

        elif cmd == "STOP":
            time.sleep(0.02)

        else:
            time.sleep(0.02)

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
        GPIO.cleanup()
    except Exception:
        pass

    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

# =========================================================
# MAIN
# =========================================================
def main():
    global motion_enabled, current_cmd

    print("Khoi dong hexapod Pi 5 AUTO VL53...")
    time.sleep(1.0)

    init_vl53()

    with pose_lock:
        writeAll()

    time.sleep(0.5)

    go_low_smooth()
    motion_enabled = False
    current_cmd = "IDLE"

    print("Robot dang LOW. Bat AUTO VL53...")

    t_motion = threading.Thread(target=motion_loop, daemon=True)
    t_motion.start()

    t_auto = threading.Thread(target=auto_loop, daemon=True)
    t_auto.start()

    while running:
        time.sleep(1)

if __name__ == "__main__":
    main()