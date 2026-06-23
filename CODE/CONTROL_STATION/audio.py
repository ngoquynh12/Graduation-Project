import os
import json
import time
import subprocess

JSON_FILE = "/tmp/lora_latest.json"

PERSON_WAV = "/home/pi/DATN/person.wav"
DOG_WAV = "/home/pi/DATN/dog.wav"

PERSON_COOLDOWN = 5
DOG_COOLDOWN = 5

last_person_time = 0
last_dog_time = 0


def play_wav(file_path):
    if not os.path.exists(file_path):
        print("[AUDIO] FILE NOT FOUND:", file_path)
        return

    subprocess.Popen(
        ["pw-play", file_path],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )


print("Bluetooth Audio Alert Started")

while True:
    try:
        with open(JSON_FILE, "r") as f:
            data = json.load(f)
    except:
        data = {}

    now = time.time()

    # Camera
    cp = int(data.get("cp", 0) or 0)   # camera person
    cd = int(data.get("cd", 0) or 0)   # camera dog

    # Microphone
    mp = int(data.get("mp", 0) or 0)   # mic person
    md = int(data.get("md", 0) or 0)   # mic dog

    print(f"cp={cp} cd={cd} mp={mp} md={md}")

    if (cp > 0 or mp == 1) and now - last_person_time > PERSON_COOLDOWN:
        print("[AUDIO] HUMAN DETECTED")
        play_wav(PERSON_WAV)
        last_person_time = now

    if (cd > 0 or md == 1) and now - last_dog_time > DOG_COOLDOWN:
        print("[AUDIO] DOG DETECTED")
        play_wav(DOG_WAV)
        last_dog_time = now

    time.sleep(0.2)