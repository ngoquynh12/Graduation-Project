import sys
import json
import os
import time
from time import sleep
import RPi.GPIO as GPIO
import threading

sys.path.append("/home/pi/DATN/pySX127x")

from SX127x.board_config import BOARD
from SX127x.LoRa import LoRa
from SX127x.constants import MODE

# =========================================================
# CONFIG
# =========================================================
OUT_FILE = "/tmp/lora_latest.json"
CMD_FILE = "/tmp/cmd_latest.json"

RST_PIN = 22
DIO0_PIN = 4

LORA_FREQ = 433.0
LORA_BW = 7               # 125 kHz
LORA_CR = 1               # 4/5
LORA_SF = 7
LORA_SYNC_WORD = 0x12
LORA_CRC = False

POLL_DELAY = 0.02
RX_SOFT_TIMEOUT = 5.0     # quá lâu không có packet thì reset RX mode
RX_HARD_TIMEOUT = 15.0    # quá lâu không có packet thì re-init radio
MAX_CONSECUTIVE_ERRORS = 8
BUZZER_PIN = 13

# =========================================================
# GPIO / BOARD CUSTOM
# =========================================================
def custom_setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(RST_PIN, GPIO.OUT)
    GPIO.output(RST_PIN, 0)
    sleep(0.1)
    GPIO.output(RST_PIN, 1)
    sleep(0.1)

    GPIO.setup(DIO0_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(BUZZER_PIN, GPIO.OUT)
    GPIO.output(BUZZER_PIN, 0)

def custom_teardown():
    try:
        if BOARD.spi is not None:
            BOARD.spi.close()
            BOARD.spi = None
    except Exception:
        pass

    try:
        GPIO.cleanup([RST_PIN, DIO0_PIN, BUZZER_PIN])
    except Exception:
        pass

BOARD.setup = custom_setup
BOARD.teardown = custom_teardown

# polling only
BOARD.add_event_detect = lambda *args, **kwargs: None
BOARD.add_events = lambda *args, **kwargs: None

# =========================================================
# HELPERS
# =========================================================
def extract_json_text(text: str):
    start = text.find("{")
    end = text.rfind("}")
    if start == -1 or end == -1 or end <= start:
        return None
    return text[start:end + 1]

def atomic_save_json(obj):
    tmp_file = OUT_FILE + ".tmp"
    with open(tmp_file, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False)
    os.replace(tmp_file, OUT_FILE)

def buzzer_on():
    GPIO.output(BUZZER_PIN, 1)

def buzzer_off():
    GPIO.output(BUZZER_PIN, 0)

def beep_stuck():
    def _beep():
        print("[ALERT] STUCK -> BUZZER ON")
        buzzer_on()
        sleep(3)
        buzzer_off()

    threading.Thread(target=_beep, daemon=True).start()

# =========================================================
# LORA RX
# =========================================================
class LoRaRcvCont(LoRa):
    def __init__(self, verbose=False):
        super(LoRaRcvCont, self).__init__(verbose)

        self.last_rx_time = time.time()
        self.last_valid_json_time = time.time()
        self.last_recover_time = 0.0
        self.consecutive_errors = 0
        self.total_packets = 0
        self.total_valid_json = 0

        self.configure_radio()

    def configure_radio(self):
        self.set_mode(MODE.SLEEP)
        sleep(0.05)

        self.set_dio_mapping([0] * 6)

        self.set_freq(LORA_FREQ)
        self.set_pa_config(pa_select=1)
        self.set_bw(LORA_BW)
        self.set_coding_rate(LORA_CR)
        self.set_spreading_factor(LORA_SF)
        self.set_rx_crc(LORA_CRC)
        self.set_sync_word(LORA_SYNC_WORD)
        self.set_agc_auto_on(True)
        self.set_low_data_rate_optim(False)

        self.clear_all_irqs()

        print("RX configured")
        print(f"freq = {LORA_FREQ}")
        print("bw = 125kHz")
        print("cr = 4/5")
        print(f"sf = {LORA_SF}")
        print(f"crc = {'on' if LORA_CRC else 'off'}")
        print(f"sync_word = {hex(LORA_SYNC_WORD)}")
        print("Version:", hex(self.get_version()))

    def clear_all_irqs(self):
        try:
            self.clear_irq_flags(
                RxDone=1,
                PayloadCrcError=1,
                ValidHeader=1,
                RxTimeout=1,
                TxDone=1,
                CadDone=1,
                FhssChangeChannel=1,
                CadDetected=1
            )
        except Exception:
            pass

    def enter_rx_mode(self):
        try:
            self.set_mode(MODE.STDBY)
            sleep(0.01)
        except Exception:
            pass

        try:
            self.reset_ptr_rx()
        except Exception:
            pass

        self.clear_all_irqs()
        self.set_mode(MODE.RXCONT)
        sleep(0.01)

    def soft_recover_rx(self, reason="unknown"):
        print(f"[RECOVER] Soft recover RX | reason={reason}")
        try:
            self.enter_rx_mode()
            self.last_recover_time = time.time()
        except Exception as e:
            print("[RECOVER] Soft recover failed:", e)

    def hard_reinit_radio(self, reason="unknown"):
        print(f"[RECOVER] Hard re-init radio | reason={reason}")
        try:
            self.set_mode(MODE.SLEEP)
        except Exception:
            pass

        sleep(0.1)
        self.configure_radio()
        self.enter_rx_mode()

        self.consecutive_errors = 0
        self.last_recover_time = time.time()

    def handle_rx_packet(self):
        self.total_packets += 1
        self.last_rx_time = time.time()

        try:
            length = self.get_rx_nb_bytes()

            if length <= 0 or length > 255:
                self.consecutive_errors += 1
                return False

            payload = self.read_payload(nocheck=True)

            if not isinstance(payload, list):
                payload = list(payload)

            payload = payload[:length]
            raw_bytes = bytes(payload)

            text = raw_bytes.decode("utf-8", errors="ignore").strip()
            if text == "ALERT:STUCK":
                self.consecutive_errors = 0
                beep_stuck()
                return True
            clean = extract_json_text(text)

            if clean is None:
                self.consecutive_errors += 1
                return False

            data = json.loads(clean)

            if not isinstance(data, dict):
                self.consecutive_errors += 1
                return False

            rssi = int(self.get_pkt_rssi_value())
            data["rssi"] = rssi

            atomic_save_json(data)

            self.total_valid_json += 1
            self.last_valid_json_time = time.time()
            self.consecutive_errors = 0

            print(f"[LORA RX] RSSI={rssi} JSON={json.dumps(data, ensure_ascii=False)}")

            return True

        except Exception as e:
            print("[LORA RX] RX read error:", e)
            self.consecutive_errors += 1
            return False

    def send_text(self, text):
        try:
            payload = list(text.encode("utf-8"))

            print("[LORA TX]", text)

            self.set_mode(MODE.STDBY)
            sleep(0.01)

            self.set_dio_mapping([1, 0, 0, 0, 0, 0])
            self.clear_all_irqs()

            self.write_payload(payload)
            self.set_mode(MODE.TX)

            timeout = time.time() + 0.5

            while True:
                irq = self.get_irq_flags()

                if irq.get("tx_done"):
                    self.clear_all_irqs()
                    self.enter_rx_mode()
                    return True

                if time.time() > timeout:
                    print("[LORA TX] TIMEOUT")
                    self.enter_rx_mode()
                    return False

                sleep(0.005)

        except Exception as e:
            print("[LORA TX] ERROR:", e)
            self.enter_rx_mode()
            return False

    def maybe_recover(self):
        now = time.time()

        # quá nhiều packet lỗi liên tiếp
        if self.consecutive_errors >= MAX_CONSECUTIVE_ERRORS:
            self.hard_reinit_radio(reason=f"consecutive_errors={self.consecutive_errors}")
            return

        # lâu không nhận được gì
        silence = now - self.last_rx_time

        if silence > RX_HARD_TIMEOUT:
            self.hard_reinit_radio(reason=f"hard_timeout={silence:.1f}s")
            self.last_rx_time = now
            return

        if silence > RX_SOFT_TIMEOUT:
            if now - self.last_recover_time > RX_SOFT_TIMEOUT:
                self.soft_recover_rx(reason=f"soft_timeout={silence:.1f}s")
                self.last_recover_time = now

    def start(self):
        print("Pi RX/TX anti-stuck polling mode...")
        self.enter_rx_mode()

        heartbeat_ts = time.time()
        last_cmd_time = 0

        while True:
            try:
                # =========================
                # SEND CMD FROM control.py
                # =========================
                try:
                    if os.path.exists(CMD_FILE):
                        with open(CMD_FILE, "r") as f:
                            cmd_data = json.load(f)

                        cmd_text = cmd_data.get("cmd", "")
                        cmd_time = float(cmd_data.get("time", 0))

                        if cmd_text and cmd_time != last_cmd_time:
                            self.send_text(cmd_text)
                            last_cmd_time = cmd_time

                except Exception as e:
                    print("[CMD READ ERROR]", e)

                # =========================
                # RECEIVE JSON FROM ROBOT
                # =========================
                irq = self.get_irq_flags()

                if irq.get("rx_done"):
                    print(">>> RX_DONE detected")

                    if irq.get("payload_crc_error"):
                        print("CRC ERROR on RX_DONE")
                        self.consecutive_errors += 1
                    else:
                        self.handle_rx_packet()

                    print("------------------------")

                    self.clear_all_irqs()
                    self.enter_rx_mode()

                elif irq.get("payload_crc_error"):
                    print("CRC ERROR")
                    self.consecutive_errors += 1
                    self.clear_all_irqs()
                    self.enter_rx_mode()

                self.maybe_recover()

                if time.time() - heartbeat_ts > 10:
                    print(
                        f"[HEARTBEAT] packets={self.total_packets} "
                        f"valid_json={self.total_valid_json} "
                        f"errors={self.consecutive_errors} "
                        f"last_rx={time.time() - self.last_rx_time:.1f}s ago"
                    )
                    heartbeat_ts = time.time()

                sleep(POLL_DELAY)

            except KeyboardInterrupt:
                raise

            except Exception as e:
                print("[LOOP ERROR]", e)
                self.consecutive_errors += 1
                sleep(0.2)
                self.soft_recover_rx(reason="loop_exception")

# =========================================================
# MAIN
# =========================================================
lora = None

try:
    BOARD.setup()
    lora = LoRaRcvCont(verbose=False)
    lora.start()

except KeyboardInterrupt:
    print("\nKeyboardInterrupt")

finally:
    try:
        if lora is not None:
            lora.set_mode(MODE.SLEEP)
    except Exception:
        pass

    BOARD.teardown()