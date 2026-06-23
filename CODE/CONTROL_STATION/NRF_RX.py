import time
import json
import os
import struct
from pyrf24 import RF24, RF24_250KBPS, RF24_PA_LOW

OUT_FILE = "/tmp/marker_latest.json"

radio = RF24(16, 10)

ADDRESS = b"MKR01"
CHANNEL = 76
TIMEOUT = 3.0

# Lưu trạng thái 3 marker
markers = {
    1: {"status": None, "battery": 0.0, "counter": 0, "last_packet_time": 0.0},
    2: {"status": None, "battery": 0.0, "counter": 0, "last_packet_time": 0.0},
    3: {"status": None, "battery": 0.0, "counter": 0, "last_packet_time": 0.0},
}


def get_status_text(s):
    return {
        0: "WAITING",
        1: "NOT RESCUED",
        2: "RESCUED"
    }.get(s, "UNKNOWN")


def get_link(marker_id):
    m = markers[marker_id]

    if m["status"] is None:
        return "LOST"

    if time.time() - m["last_packet_time"] > TIMEOUT:
        return "LOST"

    return "OK"


def save_marker_file():
    data = {}

    for marker_id in [1, 2, 3]:
        m = markers[marker_id]

        data[f"marker_{marker_id}"] = {
            "status": get_status_text(m["status"]) if m["status"] is not None else "NO DATA",
            "link": get_link(marker_id),
            "battery": round(m["battery"], 2),
            "counter": m["counter"],
            "last_packet_time": m["last_packet_time"]
        }

    tmp = OUT_FILE + ".tmp"

    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False)

    os.replace(tmp, OUT_FILE)


def init_radio():
    if not radio.begin():
        raise RuntimeError("NRF24 NOT FOUND")

    radio.setChannel(CHANNEL)
    radio.setPALevel(RF24_PA_LOW)
    radio.setDataRate(RF24_250KBPS)
    radio.openReadingPipe(1, ADDRESS)
    radio.startListening()

    print("[NRF RX] ready")
    print("[NRF RX] channel=76, addr=MKR01")


def main():
    init_radio()
    save_marker_file()

    last_refresh = 0.0

    while True:
        try:
            changed = False

            while radio.available():
                payload = radio.read(10)

                if len(payload) == 10:
                    marker_id, status, battery, counter = struct.unpack("<BBfI", payload)

                    if marker_id in markers:
                        markers[marker_id]["status"] = status
                        markers[marker_id]["battery"] = battery
                        markers[marker_id]["counter"] = counter
                        markers[marker_id]["last_packet_time"] = time.time()

                        changed = True

                        print(
                            f"[NRF RX] Marker{marker_id} "
                            f"status={status}, bat={battery:.2f}, cnt={counter}"
                        )

                    else:
                        print(f"[NRF RX] Unknown marker id={marker_id}")

            now = time.time()

            if changed:
                save_marker_file()
                last_refresh = now

            elif now - last_refresh >= 0.5:
                save_marker_file()
                last_refresh = now

            time.sleep(0.02)

        except KeyboardInterrupt:
            print("\n[NRF RX] stopped")
            break

        except Exception as e:
            print("[NRF RX] error:", e)
            time.sleep(0.2)


if __name__ == "__main__":
    main()