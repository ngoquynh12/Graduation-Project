import sys
from time import sleep
import RPi.GPIO as GPIO

sys.path.append("/home/pi/DATN/pySX127x")

from SX127x.board_config import BOARD

# ========= CHỌN CHÂN =========
RST_PIN = 22
DIO0_PIN = 4

def custom_setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # RST
    GPIO.setup(RST_PIN, GPIO.OUT)

    # HARD RESET SX1278
    GPIO.output(RST_PIN, 0)
    sleep(0.1)
    GPIO.output(RST_PIN, 1)
    sleep(0.1)

    # DIO0
    GPIO.setup(DIO0_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def custom_teardown():
    try:
        if BOARD.spi is not None:
            BOARD.spi.close()
            BOARD.spi = None
    except:
        pass
    GPIO.cleanup()

BOARD.setup = custom_setup
BOARD.teardown = custom_teardown

# dùng polling, không dùng interrupt thật
BOARD.add_event_detect = lambda *args, **kwargs: None
BOARD.add_events = lambda *args, **kwargs: None

from SX127x.LoRa import LoRa
from SX127x.constants import MODE

BOARD.setup()

class LoRaRcvCont(LoRa):
    def __init__(self, verbose=False):
        super(LoRaRcvCont, self).__init__(verbose)

        self.set_mode(MODE.SLEEP)
        sleep(0.05)

        self.set_dio_mapping([0] * 6)

        # PHẢI GIỐNG TX
        self.set_freq(433.0)
        self.set_pa_config(pa_select=1)
        self.set_bw(7)              # 125 kHz
        self.set_coding_rate(1)     # 4/5
        self.set_spreading_factor(7)
        self.set_rx_crc(False)
        self.set_sync_word(0x12)

        # clear cờ IRQ cũ
        self.clear_irq_flags(
            RxDone=1,
            PayloadCrcError=1,
            ValidHeader=1,
            RxTimeout=1,
            FhssChangeChannel=1,
            CadDone=1
        )

        print("RX configured")
        print("freq = 433.0")
        print("bw = 125kHz")
        print("cr = 4/5")
        print("sf = 7")
        print("crc = off")
        print("sync_word = 0x12")
        print("Version:", hex(self.get_version()))

    def start(self):
        print("Pi3 RX polling mode...")
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)
        sleep(0.05)

        while True:
            irq = self.get_irq_flags()

            if irq.get("rx_done"):
                print(">>> RX_DONE detected")

                payload = self.read_payload(nocheck=True)
                print("RAW:", payload)

                text = bytes(payload).decode("utf-8", "ignore")
                print("Received:", text)
                print("Packet RSSI:", self.get_pkt_rssi_value())
                print("------------------------")

                self.clear_irq_flags(
                    RxDone=1,
                    PayloadCrcError=1,
                    ValidHeader=1,
                    RxTimeout=1
                )

                self.reset_ptr_rx()
                self.set_mode(MODE.RXCONT)

            elif irq.get("payload_crc_error"):
                print("CRC ERROR")
                self.clear_irq_flags(PayloadCrcError=1, RxDone=1)
                self.reset_ptr_rx()
                self.set_mode(MODE.RXCONT)

            sleep(0.05)

lora = None

try:
    lora = LoRaRcvCont(verbose=False)
    lora.start()

except KeyboardInterrupt:
    print("\nKeyboardInterrupt")

finally:
    try:
        if lora is not None:
            lora.set_mode(MODE.SLEEP)
    except:
        pass
    BOARD.teardown()