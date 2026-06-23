import sys
import time
import pygame
import RPi.GPIO as GPIO

sys.path.append("/home/pi/DATN/pySX127x")

from SX127x.board_config import BOARD
from SX127x.LoRa import LoRa
from SX127x.constants import MODE

RST_PIN = 22
DIO0_PIN = 4

def custom_setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(RST_PIN, GPIO.OUT)
    GPIO.output(RST_PIN, 0)
    time.sleep(0.1)
    GPIO.output(RST_PIN, 1)
    time.sleep(0.1)

    GPIO.setup(DIO0_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def custom_teardown():
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

FREQ = 433.0

DEADZONE = 0.20
AXIS_TURN = 0
AXIS_MOVE = 3

BUTTON_STOP  = 0   # A
BUTTON_SLEEP = 1   # B
BUTTON_WAKE  = 2   # X
BUTTON_AUTO  = 3   # Y

SEND_INTERVAL = 0.15

MOVE_CMDS = [
    "FORWARD",
    "BACKWARD",
    "TURN_LEFT",
    "TURN_RIGHT"
]

class LoRaTx(LoRa):
    def __init__(self, verbose=False):
        super(LoRaTx, self).__init__(verbose)

        self.set_mode(MODE.SLEEP)
        time.sleep(0.05)

        self.set_dio_mapping([1, 0, 0, 0, 0, 0])

        self.set_freq(FREQ)
        self.set_pa_config(pa_select=1)
        self.set_bw(7)
        self.set_coding_rate(1)
        self.set_spreading_factor(7)
        self.set_rx_crc(False)
        self.set_sync_word(0x12)

        self.clear_irq_flags(
            TxDone=1,
            RxDone=1,
            PayloadCrcError=1,
            ValidHeader=1,
            RxTimeout=1,
            FhssChangeChannel=1,
            CadDone=1
        )

        print("TX configured")
        print("Version:", hex(self.get_version()))

    def send_text(self, text):
        payload = list(text.encode("utf-8"))

        print("Sending:", text)

        self.set_mode(MODE.STDBY)
        time.sleep(0.01)

        self.set_dio_mapping([1, 0, 0, 0, 0, 0])

        self.clear_irq_flags(TxDone=1)
        self.write_payload(payload)
        self.set_mode(MODE.TX)

        timeout = time.time() + 0.5

        while True:
            irq = self.get_irq_flags()

            if irq.get("tx_done"):
                print(">>> TX_DONE OK")
                self.clear_irq_flags(TxDone=1)
                self.set_mode(MODE.STDBY)
                return True

            if time.time() > timeout:
                print("TX REAL TIMEOUT")
                self.set_mode(MODE.STDBY)
                return False

            time.sleep(0.01)
    
def dz(v):
    return 0.0 if abs(v) < DEADZONE else v

def init_joystick():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("Khong tim thay tay cam")
        return None

    js = pygame.joystick.Joystick(0)
    js.init()
    print("Da ket noi tay cam:", js.get_name())
    print("Axes:", js.get_numaxes(), "Buttons:", js.get_numbuttons())
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
    BOARD.setup()

    lora = LoRaTx(verbose=False)
    js = init_joystick()

    if js is None:
        return

    last_cmd = None
    last_send = 0

    listen_only = False

    print("Pi3 joystick LoRa TX ready")

    try:
        while True:
            now = time.time()

            if listen_only:
                pygame.event.pump()

                # A = STOP khẩn cấp, vẫn cho gửi kể cả đang listen_only
                if js.get_button(BUTTON_STOP):
                    msg = "CMD:STOP"
                    ok = lora.send_text(msg)

                    if ok:
                        print("SEND OK:", msg)
                    else:
                        print("SEND FAIL:", msg)

                    last_cmd = "STOP"
                    last_send = now
                    time.sleep(0.2)
                    continue

                # X = WAKE thoát listen_only
                if js.get_button(BUTTON_WAKE):
                    msg = "CMD:WAKE"
                    ok = lora.send_text(msg)

                    if ok:
                        print("SEND OK:", msg)
                    else:
                        print("SEND FAIL:", msg)

                    listen_only = False
                    last_cmd = "WAKE"
                    last_send = now
                    print("WAKE -> EXIT LISTEN ONLY MODE")

                time.sleep(0.003)
                continue

            cmd = read_cmd(js)
            # Nếu vừa bấm STOP thì gửi STOP ngay
            if cmd == "STOP" and last_cmd != "STOP":
                msg = "CMD:STOP"
                for i in range(3):
                    ok = lora.send_text(msg)
                    if ok:
                        print("SEND OK:", msg)
                    else:
                        print("SEND FAIL:", msg)
                    time.sleep(0.15)

                last_cmd = "STOP"
                last_send = now
                listen_only = True
                print("STOP -> ENTER LISTEN ONLY MODE")

                time.sleep(0.02)
                continue

            should_send = False

            if cmd != last_cmd:
                should_send = True

            elif cmd in MOVE_CMDS and now - last_send >= SEND_INTERVAL:
                should_send = True

            elif cmd == "IDLE" and now - last_send >= 0.5:
                should_send = True

            if should_send:
                msg = "CMD:" + cmd
                ok = lora.send_text(msg)

                if ok:
                    print("SEND OK:", msg)
                else:
                    print("SEND FAIL:", msg)

                last_cmd = cmd
                last_send = now

            time.sleep(0.015)

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt")

    finally:
        try:
            lora.set_mode(MODE.SLEEP)
        except Exception:
            pass

        BOARD.teardown()
        pygame.quit()

if __name__ == "__main__":
    main()