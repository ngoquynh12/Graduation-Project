import time
import struct
import board
import digitalio
from PIL import Image, ImageDraw, ImageFont
from adafruit_rgb_display import ili9341
from RF24 import RF24, RF24_250KBPS, RF24_PA_LOW

# ================= LCD =================
spi_lcd = board.SPI()

cs_pin = digitalio.DigitalInOut(board.CE1)   # GPIO7
dc_pin = digitalio.DigitalInOut(board.D25)   # GPIO25
rst_pin = digitalio.DigitalInOut(board.D24)  # GPIO24

display = ili9341.ILI9341(
    spi_lcd,
    cs=cs_pin,
    dc=dc_pin,
    rst=rst_pin,
    baudrate=32000000,
    rotation=0
)

width = display.width
height = display.height

image = Image.new("RGB", (width, height), (0, 0, 0))
draw = ImageDraw.Draw(image)
font = ImageFont.load_default()

# ================= NRF24 =================
radio = RF24(16, 10)   # CE=GPIO16, CSN=SPI1.0

ADDRESS = b"MKR01"
CHANNEL = 76
TIMEOUT = 3.0

marker_status = None
last_packet_time = 0

def get_status_text(s):
    return {
        0: "WAITING",
        1: "NOT RESCUED",
        2: "RESCUED"
    }.get(s, "UNKNOWN")

def get_link():
    if marker_status is None:
        return "LOST"
    if time.time() - last_packet_time > TIMEOUT:
        return "LOST"
    return "OK"

def draw_screen():
    draw.rectangle((0, 0, width, height), fill=(0, 0, 0))

    status_text = get_status_text(marker_status) if marker_status is not None else "NO DATA"
    link_text = get_link()

    draw.text((20, 20), "Marker Monitor", font=font, fill=(255, 255, 0))
    draw.text((20, 70), f"Marker 1:", font=font, fill=(255, 255, 255))
    draw.text((20, 100), status_text, font=font, fill=(0, 255, 0))

    # màu link
    link_color = (0, 255, 0) if link_text == "OK" else (255, 0, 0)
    draw.text((20, 150), f"Link: {link_text}", font=font, fill=link_color)

    display.image(image)

# ================= INIT =================
if not radio.begin():
    raise RuntimeError("NRF24 NOT FOUND")

radio.setChannel(CHANNEL)
radio.setPALevel(RF24_PA_LOW)
radio.setDataRate(RF24_250KBPS)
radio.openReadingPipe(1, ADDRESS)
radio.startListening()

print("RX + LCD ready...")

# ================= LOOP =================
while True:
    if radio.available():
        payload = radio.read(10)

        if len(payload) == 10:
            marker_id, status, battery, counter = struct.unpack("<BBfI", payload)

            if marker_id == 1:
                marker_status = status
                last_packet_time = time.time()

                print(f"RX: status={status}, bat={battery:.2f}, cnt={counter}")

    draw_screen()
    time.sleep(0.2)