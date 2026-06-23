import sys
import time
import signal
import threading
from dataclasses import dataclass
import cv2
from ultralytics import YOLO
from picamera2 import Picamera2
import pygame
import board
import busio
from adafruit_pca9685 import PCA9685

# =========================================================
# PCA9685
# =========================================================
I2C_FREQ = 100000
PCA_LEFT_ADDR = 0x40
PCA_RIGHT_ADDR = 0x60   # n?u i2cdetect ra 0x41 th� d?i l?i ? d�y

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
    0, 0, 0, 0,
    0, 0, 0, 0,
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
# TH�NG S?
# =========================================================
MID = 300
# LOW / SLEEP pose
L_LOW_FEMUR = 355
L_LOW_TIBIA = 265
R_LOW_FEMUR = 245
R_LOW_TIBIA = 335
# lift
L_UP_FEMUR = 375
L_UP_TIBIA = 375
R_UP_FEMUR = 230
R_UP_TIBIA = 230

# bu?c ti?n/l�i
L_FWD = 350
L_BWD = 250
R_FWD = 250
R_BWD = 350

# quay tr�i/ph?i
L_TURN_FWD = 340
L_TURN_BWD = 260
R_TURN_FWD = 260
R_TURN_BWD = 340

STEP_DELAY = 0.01
STEP_SIZE = 4
LOW_STEP_DELAY = 0.025
LOW_STEP_SIZE = 1
PHASE_DELAY = 0.03
STAND_HOLD = 0.2

# =========================================================
# ROBOT STATE
# =========================================================
ROBOT_BOOTING = 0
ROBOT_STANDING = 1
ROBOT_MOVING = 2
ROBOT_LOW = 3
DIR_FORWARD = 0
DIR_BACKWARD = 1
DIR_TURN_LEFT = 2
DIR_TURN_RIGHT = 3

# =========================================================
# JOYSTICK
# =========================================================
DEADZONE = 0.20

# Mapping d� test t? tay c?m c?a b?n:
# axis 0 = tr�i/ph?i
# axis 3 = ti?n/l�i
# button 0 = A
AXIS_TURN = 0
AXIS_MOVE = 3
BUTTON_STOP = 0
BUTTON_RESUME = 1  
# =========================================================
# GLOBALS
# =========================================================
robotState = ROBOT_BOOTING
current_cmd = "IDLE"
running = True
# =========================================================
# CAMERA + MARKER
# =========================================================
YOLO_MODEL_PATH = "/home/pi/human_sound_model/best.pt"

CAM_W, CAM_H = 1640, 1232
SHOW_W, SHOW_H = 640, 480
IMG_SIZE = 320
CONF_THRESHOLD = 0.45
INFER_INTERVAL = 0.25

person_detected = False
marker_count = 0
marker_busy = False

robot_locked = False
motion_lock = threading.Lock()

# Marker servo tren PCA 0x60
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

DOOR_CLOSE = 70
DOOR_OPEN  = 0
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
    lfF: int = MID
    lfT: int = MID

    lmC: int = MID
    lmF: int = MID
    lmT: int = MID

    lrC: int = MID
    lrF: int = MID
    lrT: int = MID

    rfC: int = MID
    rfF: int = MID
    rfT: int = MID

    rmC: int = MID
    rmF: int = MID
    rmT: int = MID

    rrC: int = MID
    rrF: int = MID
    rrT: int = MID


pose = RobotPose()
# =========================================================
# LOW LEVEL PWM
# =========================================================
def pulse_to_duty_cycle(pulse: int) -> int:
    pulse = max(0, min(4095, pulse))
    return pulse << 4  # 12-bit sang 16-bit

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

def standSmooth():
    global robotState

    done = False
    while not done and running:
        done = True

        pose.lfC = moveToward(pose.lfC, MID, STEP_SIZE)
        pose.lfF = moveToward(pose.lfF, MID, STEP_SIZE)
        pose.lfT = moveToward(pose.lfT, MID, STEP_SIZE)

        pose.lmC = moveToward(pose.lmC, MID, STEP_SIZE)
        pose.lmF = moveToward(pose.lmF, MID, STEP_SIZE)
        pose.lmT = moveToward(pose.lmT, MID, STEP_SIZE)

        pose.lrC = moveToward(pose.lrC, MID, STEP_SIZE)
        pose.lrF = moveToward(pose.lrF, MID, STEP_SIZE)
        pose.lrT = moveToward(pose.lrT, MID, STEP_SIZE)

        pose.rfC = moveToward(pose.rfC, MID, STEP_SIZE)
        pose.rfF = moveToward(pose.rfF, MID, STEP_SIZE)
        pose.rfT = moveToward(pose.rfT, MID, STEP_SIZE)

        pose.rmC = moveToward(pose.rmC, MID, STEP_SIZE)
        pose.rmF = moveToward(pose.rmF, MID, STEP_SIZE)
        pose.rmT = moveToward(pose.rmT, MID, STEP_SIZE)

        pose.rrC = moveToward(pose.rrC, MID, STEP_SIZE)
        pose.rrF = moveToward(pose.rrF, MID, STEP_SIZE)
        pose.rrT = moveToward(pose.rrT, MID, STEP_SIZE)

        if (
            pose.lfC != MID or pose.lfF != MID or pose.lfT != MID or
            pose.lmC != MID or pose.lmF != MID or pose.lmT != MID or
            pose.lrC != MID or pose.lrF != MID or pose.lrT != MID or
            pose.rfC != MID or pose.rfF != MID or pose.rfT != MID or
            pose.rmC != MID or pose.rmF != MID or pose.rmT != MID or
            pose.rrC != MID or pose.rrF != MID or pose.rrT != MID
        ):
            done = False

        writeAll()
        time.sleep(STEP_DELAY)

    robotState = ROBOT_STANDING
def go_low_smooth():
    global robotState

    done = False
    while not done and running:
        done = True

        pose.lfC = moveToward(pose.lfC, MID, LOW_STEP_SIZE)
        pose.lfF = moveToward(pose.lfF, L_LOW_FEMUR, LOW_STEP_SIZE)
        pose.lfT = moveToward(pose.lfT, L_LOW_TIBIA, LOW_STEP_SIZE)

        pose.lmC = moveToward(pose.lmC, MID, LOW_STEP_SIZE)
        pose.lmF = moveToward(pose.lmF, L_LOW_FEMUR, LOW_STEP_SIZE)
        pose.lmT = moveToward(pose.lmT, L_LOW_TIBIA, LOW_STEP_SIZE)

        pose.lrC = moveToward(pose.lrC, MID, LOW_STEP_SIZE)
        pose.lrF = moveToward(pose.lrF, L_LOW_FEMUR, LOW_STEP_SIZE)
        pose.lrT = moveToward(pose.lrT, L_LOW_TIBIA, LOW_STEP_SIZE)

        pose.rfC = moveToward(pose.rfC, MID, LOW_STEP_SIZE)
        pose.rfF = moveToward(pose.rfF, R_LOW_FEMUR, LOW_STEP_SIZE)
        pose.rfT = moveToward(pose.rfT, R_LOW_TIBIA, LOW_STEP_SIZE)

        pose.rmC = moveToward(pose.rmC, MID, LOW_STEP_SIZE)
        pose.rmF = moveToward(pose.rmF, R_LOW_FEMUR, LOW_STEP_SIZE)
        pose.rmT = moveToward(pose.rmT, R_LOW_TIBIA, LOW_STEP_SIZE)

        pose.rrC = moveToward(pose.rrC, MID, LOW_STEP_SIZE)
        pose.rrF = moveToward(pose.rrF, R_LOW_FEMUR, LOW_STEP_SIZE)
        pose.rrT = moveToward(pose.rrT, R_LOW_TIBIA, LOW_STEP_SIZE)

        if (
            pose.lfF != L_LOW_FEMUR or pose.lfT != L_LOW_TIBIA or
            pose.lmF != L_LOW_FEMUR or pose.lmT != L_LOW_TIBIA or
            pose.lrF != L_LOW_FEMUR or pose.lrT != L_LOW_TIBIA or
            pose.rfF != R_LOW_FEMUR or pose.rfT != R_LOW_TIBIA or
            pose.rmF != R_LOW_FEMUR or pose.rmT != R_LOW_TIBIA or
            pose.rrF != R_LOW_FEMUR or pose.rrT != R_LOW_TIBIA
        ):
            done = False

        writeAll()
        time.sleep(LOW_STEP_DELAY)

    robotState = ROBOT_LOW
def standFirst():
    if robotState != ROBOT_STANDING:
        standSmooth()
        time.sleep(STAND_HOLD)

def phaseA(a_lf_target: int, a_rm_target: int, a_lr_target: int,
           b_rf_target: int, b_lm_target: int, b_rr_target: int):
    done = False

    while not done and running:
        done = True

        pose.lfF = moveToward(pose.lfF, L_UP_FEMUR, STEP_SIZE)
        pose.lfT = moveToward(pose.lfT, L_UP_TIBIA, STEP_SIZE)

        pose.rmF = moveToward(pose.rmF, R_UP_FEMUR, STEP_SIZE)
        pose.rmT = moveToward(pose.rmT, R_UP_TIBIA, STEP_SIZE)

        pose.lrF = moveToward(pose.lrF, L_UP_FEMUR, STEP_SIZE)
        pose.lrT = moveToward(pose.lrT, L_UP_TIBIA, STEP_SIZE)

        if (
            pose.lfF != L_UP_FEMUR or pose.lfT != L_UP_TIBIA or
            pose.rmF != R_UP_FEMUR or pose.rmT != R_UP_TIBIA or
            pose.lrF != L_UP_FEMUR or pose.lrT != L_UP_TIBIA
        ):
            done = False

        writeAll()
        time.sleep(STEP_DELAY)

    done = False
    while not done and running:
        done = True

        pose.lfC = moveToward(pose.lfC, a_lf_target, STEP_SIZE)
        pose.rmC = moveToward(pose.rmC, a_rm_target, STEP_SIZE)
        pose.lrC = moveToward(pose.lrC, a_lr_target, STEP_SIZE)

        pose.rfC = moveToward(pose.rfC, b_rf_target, STEP_SIZE)
        pose.lmC = moveToward(pose.lmC, b_lm_target, STEP_SIZE)
        pose.rrC = moveToward(pose.rrC, b_rr_target, STEP_SIZE)

        if (
            pose.lfC != a_lf_target or pose.rmC != a_rm_target or pose.lrC != a_lr_target or
            pose.rfC != b_rf_target or pose.lmC != b_lm_target or pose.rrC != b_rr_target
        ):
            done = False

        writeAll()
        time.sleep(STEP_DELAY)

    done = False
    while not done and running:
        done = True

        pose.lfF = moveToward(pose.lfF, MID, STEP_SIZE)
        pose.lfT = moveToward(pose.lfT, MID, STEP_SIZE)

        pose.rmF = moveToward(pose.rmF, MID, STEP_SIZE)
        pose.rmT = moveToward(pose.rmT, MID, STEP_SIZE)

        pose.lrF = moveToward(pose.lrF, MID, STEP_SIZE)
        pose.lrT = moveToward(pose.lrT, MID, STEP_SIZE)

        if (
            pose.lfF != MID or pose.lfT != MID or
            pose.rmF != MID or pose.rmT != MID or
            pose.lrF != MID or pose.lrT != MID
        ):
            done = False

        writeAll()
        time.sleep(STEP_DELAY)
def phaseB(b_rf_target: int, b_lm_target: int, b_rr_target: int,
           a_lf_target: int, a_rm_target: int, a_lr_target: int):
    done = False

    while not done and running:
        done = True

        pose.rfF = moveToward(pose.rfF, R_UP_FEMUR, STEP_SIZE)
        pose.rfT = moveToward(pose.rfT, R_UP_TIBIA, STEP_SIZE)

        pose.lmF = moveToward(pose.lmF, L_UP_FEMUR, STEP_SIZE)
        pose.lmT = moveToward(pose.lmT, L_UP_TIBIA, STEP_SIZE)

        pose.rrF = moveToward(pose.rrF, R_UP_FEMUR, STEP_SIZE)
        pose.rrT = moveToward(pose.rrT, R_UP_TIBIA, STEP_SIZE)

        if (
            pose.rfF != R_UP_FEMUR or pose.rfT != R_UP_TIBIA or
            pose.lmF != L_UP_FEMUR or pose.lmT != L_UP_TIBIA or
            pose.rrF != R_UP_FEMUR or pose.rrT != R_UP_TIBIA
        ):
            done = False

        writeAll()
        time.sleep(STEP_DELAY)

    done = False
    while not done and running:
        done = True

        pose.rfC = moveToward(pose.rfC, b_rf_target, STEP_SIZE)
        pose.lmC = moveToward(pose.lmC, b_lm_target, STEP_SIZE)
        pose.rrC = moveToward(pose.rrC, b_rr_target, STEP_SIZE)

        pose.lfC = moveToward(pose.lfC, a_lf_target, STEP_SIZE)
        pose.rmC = moveToward(pose.rmC, a_rm_target, STEP_SIZE)
        pose.lrC = moveToward(pose.lrC, a_lr_target, STEP_SIZE)

        if (
            pose.rfC != b_rf_target or pose.lmC != b_lm_target or pose.rrC != b_rr_target or
            pose.lfC != a_lf_target or pose.rmC != a_rm_target or pose.lrC != a_lr_target
        ):
            done = False

        writeAll()
        time.sleep(STEP_DELAY)

    done = False
    while not done and running:
        done = True

        pose.rfF = moveToward(pose.rfF, MID, STEP_SIZE)
        pose.rfT = moveToward(pose.rfT, MID, STEP_SIZE)

        pose.lmF = moveToward(pose.lmF, MID, STEP_SIZE)
        pose.lmT = moveToward(pose.lmT, MID, STEP_SIZE)

        pose.rrF = moveToward(pose.rrF, MID, STEP_SIZE)
        pose.rrT = moveToward(pose.rrT, MID, STEP_SIZE)

        if (
            pose.rfF != MID or pose.rfT != MID or
            pose.lmF != MID or pose.lmT != MID or
            pose.rrF != MID or pose.rrT != MID
        ):
            done = False

        writeAll()
        time.sleep(STEP_DELAY)
def moveRobot(direction: int, steps: int):
    global robotState
    standFirst()
    robotState = ROBOT_MOVING

    for _ in range(steps):
        if not running:
            return

        if direction == DIR_FORWARD:
            phaseA(L_FWD, R_FWD, L_FWD, R_BWD, L_BWD, R_BWD)
            time.sleep(PHASE_DELAY)
            phaseB(R_FWD, L_FWD, R_FWD, L_BWD, R_BWD, L_BWD)
            time.sleep(PHASE_DELAY)

        elif direction == DIR_BACKWARD:
            phaseA(L_BWD, R_BWD, L_BWD, R_FWD, L_FWD, R_FWD)
            time.sleep(PHASE_DELAY)
            phaseB(R_BWD, L_BWD, R_BWD, L_FWD, R_FWD, L_FWD)
            time.sleep(PHASE_DELAY)

        elif direction == DIR_TURN_LEFT:
            phaseA(L_TURN_BWD, R_TURN_FWD, L_TURN_BWD,
                   R_TURN_BWD, L_TURN_FWD, R_TURN_BWD)
            time.sleep(PHASE_DELAY)
            phaseB(R_TURN_FWD, L_TURN_BWD, R_TURN_FWD,
                   L_TURN_FWD, R_TURN_BWD, L_TURN_FWD)
            time.sleep(PHASE_DELAY)

        elif direction == DIR_TURN_RIGHT:
            phaseA(L_TURN_FWD, R_TURN_BWD, L_TURN_FWD,
                   R_TURN_FWD, L_TURN_BWD, R_TURN_FWD)
            time.sleep(PHASE_DELAY)
            phaseB(R_TURN_BWD, L_TURN_FWD, R_TURN_BWD,
                   L_TURN_BWD, R_TURN_FWD, L_TURN_BWD)
            time.sleep(PHASE_DELAY)

    standSmooth()
    time.sleep(STAND_HOLD)
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

def marker_open_door():
    set_marker_servo(MARKER_DOOR_SERVO, DOOR_OPEN)
    time.sleep(0.8)

def marker_close_door():
    set_marker_servo(MARKER_DOOR_SERVO, DOOR_CLOSE)
    time.sleep(0.8)

def drop_marker_sequence():
    global marker_busy
    global marker_count

    if marker_busy:
        return

    if marker_count >= 3:
        return

    marker_busy = True
    marker_count += 1
    n = marker_count

    try:
        print(f"[MARKER] DROP MARKER {n}")

        if n == 1:
            marker_open_door()
            set_marker_servo(MARKER_PUSH_SERVO_1, PUSH1_HALF)
            time.sleep(1.2)
            marker_close_door()

        elif n == 2:
            marker_open_door()
            set_marker_servo(MARKER_PUSH_SERVO_1, PUSH1_OUT)
            time.sleep(1.2)
            set_marker_servo(MARKER_PUSH_SERVO_2, PUSH2_SMALL)
            time.sleep(1.2)
            marker_close_door()

        elif n == 3:
            marker_open_door()

            set_marker_servo(MARKER_PUSH_SERVO_2, PUSH2_OUT)
            time.sleep(1.2)

            # Thu servo về hết trước
            set_marker_servo(MARKER_PUSH_SERVO_2, PUSH2_HOME)
            time.sleep(1.2)

            set_marker_servo(MARKER_PUSH_SERVO_1, PUSH1_HOME)
            time.sleep(1.2)

             # Sau đó mới đóng cửa
            marker_close_door()

    except Exception as e:
        print("[MARKER ERROR]", e)

    finally:
        marker_busy = False
def camera_loop():

    global person_detected
    global current_cmd
    global robot_locked
    global running
    global marker_count
    global marker_busy

    print("[CAMERA] Loading YOLO...")

    model = YOLO(YOLO_MODEL_PATH)

    print("[CAMERA] YOLO ready")

    picam2 = Picamera2()

    config = picam2.create_preview_configuration(
        main={
            "size": (CAM_W, CAM_H),
            "format": "RGB888"
        },
        raw={
            "size": (CAM_W, CAM_H)
        }
    )

    picam2.configure(config)

    picam2.start()

    last_infer = 0

    try:

        while running:

            frame = picam2.capture_array()

            frame_small = cv2.resize(
                frame,
                (SHOW_W, SHOW_H)
            )

            now = time.time()

            found_person = False

            # =====================================
            # YOLO INFERENCE
            # =====================================

            if now - last_infer >= INFER_INTERVAL:

                last_infer = now

                results = model.predict(
                    frame_small,
                    imgsz=IMG_SIZE,
                    conf=CONF_THRESHOLD,
                    verbose=False
                )

                r0 = results[0]

                if (
                    r0.boxes is not None
                    and
                    len(r0.boxes) > 0
                ):

                    clss = (
                        r0.boxes.cls
                        .cpu()
                        .numpy()
                        .astype(int)
                    )

                    for c in clss:

                        label = r0.names.get(
                            int(c),
                            str(int(c))
                        )

                        if label in ["person", "human"]:

                            found_person = True

                            break

                # =====================================
                # SHOW YOLO RESULT
                # =====================================

                show = r0.plot()

            else:

                show = frame_small

            person_detected = found_person

            # =====================================
            # PERSON DETECTED
            # =====================================

            if (
                found_person
                and
                marker_count < 3
                and
                not marker_busy
                and
                not robot_locked
               ):

                print(
                    "[CAMERA] PERSON DETECTED "
                    "-> STOP, SLEEP, DROP MARKER"
                )

                robot_locked = True

                current_cmd = "IDLE"

                with motion_lock:

                     print("[ROBOT] STAND BEFORE LOW")

                     standSmooth()

                     time.sleep(0.7)

                     print("[ROBOT] GO LOW SLOWLY")

                     go_low_smooth()

                     time.sleep(1.0)

                     drop_marker_sequence()

            # =====================================
            # SHOW CAMERA
            # =====================================

            cv2.imshow(
                "YOLO Camera",
                show
            )

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):

                running = False

                break

            time.sleep(0.01)

    except Exception as e:

        print("[CAMERA ERROR]", e)

    finally:

        try:
            picam2.stop()
        except Exception:
            pass

        cv2.destroyAllWindows() 
# =========================================================
# JOYSTICK
# =========================================================
def init_joystick():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("Khong tim thay tay cam USB")
        return None

    js = pygame.joystick.Joystick(0)
    js.init()
    print("Da ket noi tay cam:", js.get_name())
    print("Axes:", js.get_numaxes(), "Buttons:", js.get_numbuttons())
    return js

def dz(v: float) -> float:
    return 0.0 if abs(v) < DEADZONE else v

def read_command(js) -> str:
    pygame.event.pump()

    lx = dz(js.get_axis(AXIS_TURN))
    ly = dz(js.get_axis(AXIS_MOVE))
    stop_btn = js.get_button(BUTTON_STOP)

    if stop_btn:
        return "STOP"
    if ly < -0.5:
        return "FORWARD"
    if ly > 0.5:
        return "BACKWARD"
    if lx < -0.5:
        return "TURN_LEFT"
    if lx > 0.5:
        return "TURN_RIGHT"
    return "IDLE"

# =========================================================
# THREAD �I?U KHI?N
# =========================================================
def motion_loop():
    global current_cmd

    while running:
        if robot_locked:
            time.sleep(0.05)
            continue

        cmd = current_cmd

        with motion_lock:
            if cmd == "FORWARD":
                moveRobot(DIR_FORWARD, 1)
            elif cmd == "BACKWARD":
                moveRobot(DIR_BACKWARD, 1)
            elif cmd == "TURN_LEFT":
                moveRobot(DIR_TURN_LEFT, 1)
            elif cmd == "TURN_RIGHT":
                moveRobot(DIR_TURN_RIGHT, 1)
            elif cmd == "STOP":
                standSmooth()
                time.sleep(0.05)
            else:
                time.sleep(0.05)

def joystick_loop(js):
    global current_cmd
    global robot_locked
   

    last_print = None

    while running:
        pygame.event.pump()

        # Nếu robot đang LOW sau khi thả marker
        if robot_locked:
            current_cmd = "IDLE"

            resume_btn = js.get_button(BUTTON_RESUME)

            if resume_btn:
                print("[RESUME] STAND UP SAFELY")

                with motion_lock:
                    standSmooth()
                    time.sleep(1.0)

                robot_locked = False

                # không reset marker_count để lần sau thả marker tiếp theo
                print("[RESUME] ROBOT READY")

            time.sleep(0.05)
            continue

        cmd = read_command(js)
        current_cmd = cmd

        if cmd != last_print:
            print("CMD =", cmd)
            last_print = cmd

        time.sleep(0.03)
# =========================================================
# CLEANUP
# =========================================================
def cleanup(*_):
    global running
    running = False
    print("\nDang dung robot an toan...")

    try:
        go_low_smooth()
        writeAll()
    except Exception as e:
        print("Cleanup error:", e)

    try:
        pcaL.deinit()
        pcaR.deinit()
    except Exception:
        pass

    try:
        pygame.quit()
    except Exception:
        pass

    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

# =========================================================
# MAIN
# =========================================================
def main():
    global robotState

    print("Khoi dong hexapod Pi 5...")
    time.sleep(1.0)

    writeAll()
    time.sleep(0.5)

    standSmooth()
    time.sleep(1.0)
    marker_init()
    
    js = init_joystick()
    if js is None:
        while True:
            time.sleep(1)

    t_motion = threading.Thread(target=motion_loop, daemon=True)
    t_motion.start()
    t_camera = threading.Thread(target=camera_loop, daemon=True)
    t_camera.start()

    joystick_loop(js)

if __name__ == "__main__":
    main()  