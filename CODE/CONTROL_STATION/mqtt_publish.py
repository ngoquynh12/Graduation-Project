import json
import time
import ssl
import paho.mqtt.client as mqtt

JSON_FILE = "/tmp/lora_latest.json"
MARKER_FILE = "/tmp/marker_latest.json"

BROKER = "11335f4271be4c3b9457e9056edf260b.s1.eu.hivemq.cloud"
PORT = 8883

USERNAME = "DATN_HK252"
PASSWORD = "12345678aA"

TOPIC = "rescue_robot/data"

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

client.username_pw_set(
    USERNAME,
    PASSWORD
)

client.tls_set(
    tls_version=ssl.PROTOCOL_TLS_CLIENT
)

client.connect(BROKER, PORT)

client.loop_start()

print("MQTT CONNECTED")

while True:

    try:
        with open(JSON_FILE, "r") as f:
            data = json.load(f)
    except:
        data = {}

    try:
        with open(MARKER_FILE, "r") as f:
            marker = json.load(f)
    except:
        marker = {}

    payload = {
        "robot": data,
        "marker": marker,
        "time": time.strftime("%H:%M:%S")
    }

    client.publish(
        TOPIC,
        json.dumps(payload)
    )

    print("SEND:", payload)

    time.sleep(1)