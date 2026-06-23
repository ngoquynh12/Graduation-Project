from LD2410 import *
from gpiozero import LED
import time
import logging
from collections import deque

# T?t to�n b? log debug c?a thu vi?n
logging.disable(logging.CRITICAL)

PORT = "/dev/ttyAMA0"
LED_PIN = 23

MAX_MOVING_DIST_CM = 120
MIN_MOVING_ENERGY = 45
HOLD_ON_COUNT = 3
HOLD_OFF_COUNT = 8
AVG_LEN = 5

led = LED(LED_PIN)

on_counter = 0
off_counter = 0
human_present = False

mov_dist_hist = deque(maxlen=AVG_LEN)
mov_energy_hist = deque(maxlen=AVG_LEN)

last_human = None
last_distance = None

radar = LD2410(port=PORT)
radar.enable_engineering_mode()
radar.start()

def avg(values):
    return int(sum(values) / len(values)) if values else 0

print("Radar running... Ctrl+C to stop")

try:
    while True:
        data = radar.get_data()

        if data and data[0]:
            std = data[0]

            detect_type = std[0]
            moving_dist_cm = std[1]
            moving_energy = std[2]

            mov_dist_hist.append(moving_dist_cm)
            mov_energy_hist.append(moving_energy)

            mov_dist_smooth = avg(mov_dist_hist)
            mov_energy_smooth = avg(mov_energy_hist)

            detected = (
                detect_type in [1, 3] and
                mov_energy_smooth >= MIN_MOVING_ENERGY and
                mov_dist_smooth <= MAX_MOVING_DIST_CM
            )

            if detected:
                on_counter += 1
                off_counter = 0
            else:
                off_counter += 1
                on_counter = 0

            if on_counter >= HOLD_ON_COUNT and not human_present:
                human_present = True
                led.on()

            if off_counter >= HOLD_OFF_COUNT and human_present:
                human_present = False
                led.off()

            # Ch? l?y kho?ng c�ch khi th?c s? c� ngu?i
            display_distance = mov_dist_smooth if human_present else 0

            # Ch? in khi tr?ng th�i thay d?i
            if human_present != last_human or display_distance != last_distance:
                print(f"HUMAN: {'YES' if human_present else 'NO '} | DISTANCE: {display_distance} cm")
                last_human = human_present
                last_distance = display_distance

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopping radar...")
    radar.stop()
    led.off()