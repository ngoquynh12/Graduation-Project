import sys
from time import sleep
import RPi.GPIO as GPIO

sys.path.append("/home/pi/human_sound_model/pySX127x")

from SX127x.board_config import BOARD

def custom_setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # RST = GPIO22
    GPIO.setup(22, GPIO.OUT)
    GPIO.output(22, 1)

    # DIO0 = GPIO26
    GPIO.setup(26, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def custom_teardown():
    GPIO.cleanup()
    if BOARD.spi is not None:
        BOARD.spi.close()
        BOARD.spi = None

BOARD.setup = custom_setup
BOARD.teardown = custom_teardown

# polling mode, kh�ng d�ng interrupt th?t
BOARD.add_event_detect = lambda *args, **kwargs: None
BOARD.add_events = lambda *args, **kwargs: None

from SX127x.LoRa import LoRa
from SX127x.constants import MODE

BOARD.setup()

class LoRaTxPolling(LoRa):
    def __init__(self, verbose=False):
        super(LoRaTxPolling, self).__init__(verbose)

        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([0] * 6)

        self.set_freq(433.0)
        self.set_pa_config(pa_select=1)
        self.set_bw(7)              # 125 kHz
        self.set_coding_rate(1)     # 4/5
        self.set_spreading_factor(7)
        self.set_rx_crc(False)
        self.set_sync_word(0x12)

        print("TX configured")
        print("freq = 433.0")
        print("bw = 125kHz")
        print("cr = 4/5")
        print("sf = 7")
        print("crc = off")
        print("sync_word = 0x12")

    def send_text(self, text):
        payload = list(text.encode("utf-8"))

        print("Sending:", text)

        self.set_mode(MODE.STDBY)
        self.clear_irq_flags(TxDone=1)
        self.write_payload(payload)
        self.set_mode(MODE.TX)

        while True:
            irq = self.get_irq_flags()
            if irq["tx_done"]:
                print(">>> TX_DONE detected")
                self.clear_irq_flags(TxDone=1)
                self.set_mode(MODE.STDBY)
                break
            sleep(0.05)

lora = LoRaTxPolling(verbose=False)

try:
    counter = 0
    while True:
        msg = f"Hello from Pi5 TX #{counter}"
        lora.send_text(msg)
        counter += 1
        sleep(2)

except KeyboardInterrupt:
    print("\nKeyboardInterrupt")
finally:
    lora.set_mode(MODE.SLEEP)
    BOARD.teardown()