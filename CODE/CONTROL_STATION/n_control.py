import time
import json
import os
import pygame

CMD_FILE = "/tmp/cmd_latest.json"

DEADZONE = 0.20
AXIS_TURN = 0
AXIS_MOVE = 3

BUTTON_STOP  = 0
BUTTON_SLEEP = 1
BUTTON_WAKE  = 2
BUTTON_AUTO  = 3

SEND_INTERVAL = 0.15

MOVE_CMDS = ["FORWARD", "BACKWARD", "TURN_LEFT", "TURN_RIGHT"]

def dz(v):
    return 0.0 if abs(v) < DEADZONE else v

def save_cmd(cmd):
    tmp = CMD_FILE + ".tmp"
    data = {
        "cmd": "CMD:" + cmd,
        "time": time.time()
    }

    with open(tmp, "w") as f:
        json.dump(data, f)

    os.replace(tmp, CMD_FILE)
    print("CMD WRITE:", data["cmd"])

def init_joystick():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("Khong tim thay tay cam")
        return None

    js = pygame.joystick.Joystick(0)
    js.init()

    print("Da ket noi tay cam:", js.get_name())
    return js

def read_cmd(js):
    pygame.event.pump()

    lx = dz(js.get_axis(AXIS_TURN))
    ly = dz(js.get_axis(AXIS_MOVE))

    if js.get_button(BUTTON_AUTO):
        return "AUTO"

    if js.get_button(BUTTON_SLEEP):
        return "SLEEP"

    if js.get_button(BUTTON_WAKE):
        return "WAKE"

    if js.get_button(BUTTON_STOP):
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

def main():
    js = init_joystick()

    if js is None:
        return

    last_cmd = None
    last_send = 0

    print("control.py ready: joystick -> /tmp/cmd_latest.json")

    try:
        while True:
            now = time.time()
            cmd = read_cmd(js)

            # Lệnh đặc biệt: gửi lặp để tránh mất gói
            if cmd in ["WAKE", "SLEEP", "STOP", "AUTO"]:
                for _ in range(5):
                    save_cmd(cmd)
                    time.sleep(0.12)

                last_cmd = cmd
                last_send = time.time()
                time.sleep(0.3)
                continue

            # Chỉ gửi IDLE 1 lần khi vừa thả nút di chuyển
            if cmd == "IDLE" and last_cmd in MOVE_CMDS:
                save_cmd("IDLE")
                last_cmd = "IDLE"
                last_send = now
                time.sleep(0.05)
                continue

            should_write = False

            if cmd in MOVE_CMDS:
                if cmd != last_cmd:
                    should_write = True
                elif now - last_send >= SEND_INTERVAL:
                    should_write = True

            # Không gửi IDLE định kỳ nữa

            if should_write:
                save_cmd(cmd)
                last_cmd = cmd
                last_send = now

            time.sleep(0.015)

    except KeyboardInterrupt:
        print("KeyboardInterrupt")

    finally:
        pygame.quit()

if __name__ == "__main__":
    main()